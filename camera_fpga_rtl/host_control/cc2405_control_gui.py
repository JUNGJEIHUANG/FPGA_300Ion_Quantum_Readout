#!/usr/bin/env python3
"""Simple GUI for the official CC2405 top.bit control path.

Features:
  * apply the canonical STD/HDR FPGA and sensor configuration;
  * write/read/compare sensor registers;
  * write FPGA registers (official hardware readback is not exposed);
  * capture and display an official-format image;
  * run the validated STD P95 automatic-exposure loop.

Close Gpixel Cobra before connecting.  Hardware operations run in a worker
thread so Tk remains responsive.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import threading
import time
import traceback
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Callable

import numpy as np
from PIL import Image, ImageTk

from ae_capture_std import apply_gain, decode_first_frame
from auto_exposure_std import (
    AEConfig,
    AEState,
    _quantize_exposure,
    encode_exposure_registers,
    select_gain,
    supported_gain_settings,
    update_ae,
)
from cc2405_official_control import build_mode_operations, run_operations
from cc2405_official_usb import (
    DEFAULT_DLL,
    Fx3Session,
    capture_frame,
    contiguous_runs,
    parse_mem,
    sensor_read,
    sensor_write,
    write_fpga,
)


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = BASE_DIR / "Test_images" / "T_GUI_capture"


def parse_int(text: str) -> int:
    return int(text.strip(), 0)


def decode_image(raw: bytes, info: dict[str, int]) -> np.ndarray:
    """Decode the first frame without changing its sensor code values."""
    if info["bytes_per_pixel"] == 1:
        dtype = np.uint8
    elif info["bytes_per_pixel"] == 2:
        dtype = np.dtype("<u2")
    else:
        raise ValueError(f"unsupported bytes/pixel: {info['bytes_per_pixel']}")
    count = info["rows"] * info["columns"]
    if len(raw) < count * info["bytes_per_pixel"]:
        raise RuntimeError("captured pixel buffer is shorter than one frame")
    return np.frombuffer(raw, dtype=dtype, count=count).reshape(info["rows"], info["columns"])


def display_image(image: np.ndarray, pixel_width: int) -> Image.Image:
    """Create an 8-bit preview; saved PNG data remains full precision."""
    shift = max(0, pixel_width - 8)
    return Image.fromarray((image.astype(np.uint16) >> shift).clip(0, 255).astype(np.uint8), "L")


def save_capture(raw: bytes, image: np.ndarray, info: dict[str, int], output_dir: Path,
                 stem: str) -> tuple[Path, Path]:
    if not stem.startswith("T_"):
        raise ValueError("generated image stem must start with T_")
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / f"{stem}.bin"
    png_path = output_dir / f"{stem}.png"
    raw_path.write_bytes(raw)
    if image.dtype == np.uint8:
        Image.fromarray(image, "L").save(png_path)
    else:
        shift = max(0, 16 - info["pixel_width"])
        Image.fromarray((image.astype(np.uint16) << shift), "I;16").save(png_path)
    return raw_path, png_path


class GuiFx3Session(Fx3Session):
    """Fx3Session adapter compatible with control.run_operations()."""

    def write(self, data: bytes) -> int:
        super().write(data)
        return len(data)


@dataclass(frozen=True)
class CaptureResult:
    raw: bytes
    image: np.ndarray
    info: dict[str, int]
    raw_path: Path
    png_path: Path


@dataclass(frozen=True)
class AutoResult:
    capture: CaptureResult
    records: tuple[dict[str, object], ...]
    converged: bool


class CC2405Backend:
    """Serialized official-protocol hardware operations used by the GUI."""

    def __init__(self) -> None:
        self.session: GuiFx3Session | None = None
        self.lock = threading.RLock()
        self.dll_path = DEFAULT_DLL
        self.device = 0
        self.timeout_ms = 30_000

    @property
    def connected(self) -> bool:
        return self.session is not None

    def connect(self, dll_path: Path, device: int, timeout_ms: int) -> str:
        with self.lock:
            self.disconnect()
            self.session = GuiFx3Session(dll_path, device, timeout_ms)
            self.dll_path, self.device, self.timeout_ms = dll_path, device, timeout_ms
            return f"Connected: device={device}, DLL={dll_path}"

    def disconnect(self) -> str:
        with self.lock:
            if self.session is not None:
                self.session.close()
                self.session = None
            return "Disconnected"

    def require_session(self) -> GuiFx3Session:
        if self.session is None:
            raise RuntimeError("not connected; close Cobra and click Connect first")
        return self.session

    def apply_mode(self, mode: str) -> str:
        with self.lock:
            session = self.require_session()
            operations = build_mode_operations(mode)
            training = operations.pop()
            power_sequence = operations.pop()
            run_operations(session, operations)
            run_operations(session, [power_sequence])
            time.sleep(0.3 if mode.lower() == "std" else 1.0)
            values = run_operations(session, [training])
            channels = [values[i:i + 7] for i in range(0, len(values), 7)]
            failed = [channel for channel in channels if len(channel) == 7 and not channel[6]]
            if failed:
                raise RuntimeError(f"{mode.upper()} training failed: {failed}")
            return f"{mode.upper()} applied; training channels={len(channels)}, failed=0"

    def write_sensor_values(self, values: list[tuple[int, int]], verify: bool) -> str:
        if not values:
            raise ValueError("no sensor register values supplied")
        with self.lock:
            sensor_write(self.require_session(), values, verify=verify)
        return f"Sensor write OK: {len(values)} ordered values; verify={verify}"

    def read_sensor_values(self, address: int, length: int) -> list[int]:
        if not 0 <= address <= 0xFFFF:
            raise ValueError("sensor address must be 0..0xFFFF")
        if not 1 <= length <= 0x3FFF:
            raise ValueError("sensor read length must be 1..16383")
        with self.lock:
            return sensor_read(self.require_session(), address, length)

    def compare_sensor_values(self, values: list[tuple[int, int]]) -> list[tuple[int, int, int, bool]]:
        """Compare hardware against the final requested value at each address."""
        final = dict(values)
        ordered = sorted(final.items())
        actual: dict[int, int] = {}
        with self.lock:
            session = self.require_session()
            for start, length in contiguous_runs(ordered):
                got = sensor_read(session, start, length)
                actual.update((start + i, value) for i, value in enumerate(got))
        return [(address, expected, actual[address], expected == actual[address])
                for address, expected in ordered]

    def write_fpga_value(self, address: int, value: int, ack: bool) -> str:
        if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFF:
            raise ValueError("FPGA address/value must be 0..0xFFFF")
        with self.lock:
            write_fpga(self.require_session(), [(address, value)], ack=ack)
        return f"FPGA write OK: 0x{address:04X}=0x{value:04X}; ACK={ack}"

    def capture(self, output_dir: Path, stem: str | None = None) -> CaptureResult:
        with self.lock:
            raw, info = capture_frame(self.require_session(), self.timeout_ms)
        image = decode_image(raw, info)
        if stem is None:
            stem = datetime.now().strftime("T_GUI_%Y%m%d_%H%M%S_%f")
        raw_path, png_path = save_capture(raw, image, info, output_dir, stem)
        return CaptureResult(raw, image, info, raw_path, png_path)

    def auto_expose(self, output_dir: Path, *, initial_exposure_us: float,
                    min_exposure_us: float, max_exposure_us: float,
                    initial_gain: float, min_gain: float, max_gain: float,
                    iterations: int, tolerance: float, settling_frames: int) -> AutoResult:
        cfg = AEConfig(min_exposure_us=min_exposure_us, max_exposure_us=max_exposure_us,
                       min_analog_gain=min_gain, max_analog_gain=max_gain)
        gains = supported_gain_settings(min_gain, max_gain)
        gain_setting = select_gain(gains, initial_gain)
        state = AEState(_quantize_exposure(initial_exposure_us, cfg), gain_setting.analog_gain)
        records: list[dict[str, object]] = []
        final: CaptureResult | None = None
        converged = False
        run_tag = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        with self.lock:
            session = self.require_session()
            write_fpga(session, list(encode_exposure_registers(state.exposure_us, cfg.line_time_us)))
            apply_gain(session, gain_setting)
            for _ in range(settling_frames):
                capture_frame(session, self.timeout_ms)

            for iteration in range(1, iterations + 1):
                raw, info = capture_frame(session, self.timeout_ms)
                image = decode_first_frame(raw, info)
                result = update_ae(image, state, cfg)
                relative_error = abs(result.p95 - cfg.target_p95) / cfg.target_p95
                stem = f"T_GUI_AE_{run_tag}_iter{iteration:02d}"
                raw_path, png_path = save_capture(raw, image, info, output_dir, stem)
                final = CaptureResult(raw, image, info, raw_path, png_path)
                converged = relative_error <= tolerance and result.saturated_pixels == 0
                records.append({
                    "iteration": iteration,
                    "p95": result.p95,
                    "saturated": result.saturated_pixels,
                    "exposure_us": state.exposure_us,
                    "gain": state.analog_gain,
                    "next_exposure_us": result.exposure_us,
                    "next_gain": result.analog_gain,
                    "reason": "CONVERGED" if converged else result.reason,
                })
                if converged:
                    break
                if result.exposure_us != state.exposure_us:
                    write_fpga(session, list(result.fpga_exposure_registers))
                if result.gain_setting.analog_gain != state.analog_gain:
                    apply_gain(session, result.gain_setting)
                state = AEState(result.exposure_us, result.analog_gain)

            if final is None:
                raise RuntimeError("automatic exposure produced no frame")
            if not converged:
                # The iteration limit bounds parameter updates.  Verify the
                # last applied update without performing an extra update.
                raw, info = capture_frame(session, self.timeout_ms)
                image = decode_first_frame(raw, info)
                check = update_ae(image, state, cfg)
                relative_error = abs(check.p95 - cfg.target_p95) / cfg.target_p95
                converged = relative_error <= tolerance and check.saturated_pixels == 0
                stem = f"T_GUI_AE_{run_tag}_final_check"
                raw_path, png_path = save_capture(raw, image, info, output_dir, stem)
                final = CaptureResult(raw, image, info, raw_path, png_path)
                records.append({
                    "iteration": "final_check",
                    "p95": check.p95,
                    "saturated": check.saturated_pixels,
                    "exposure_us": state.exposure_us,
                    "gain": state.analog_gain,
                    "next_exposure_us": state.exposure_us,
                    "next_gain": state.analog_gain,
                    "reason": "CONVERGED" if converged else "iteration limit reached",
                })
        return AutoResult(final, tuple(records), converged)


class CC2405Gui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("CC2405 Sensor / FPGA Control")
        self.geometry("1120x780")
        self.minsize(960, 680)
        self.backend = CC2405Backend()
        self.busy = False
        self.preview_photo: ImageTk.PhotoImage | None = None
        self._build_variables()
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build_variables(self) -> None:
        self.dll_var = tk.StringVar(value=str(DEFAULT_DLL))
        self.device_var = tk.StringVar(value="0")
        self.timeout_var = tk.StringVar(value="30000")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.sensor_addr_var = tk.StringVar(value="0x00A0")
        self.sensor_data_var = tk.StringVar(value="0x01")
        self.sensor_len_var = tk.StringVar(value="2")
        self.sensor_file_var = tk.StringVar()
        self.sensor_verify_var = tk.BooleanVar(value=True)
        self.fpga_addr_var = tk.StringVar(value="0x0103")
        self.fpga_data_var = tk.StringVar(value="0x0001")
        self.fpga_ack_var = tk.BooleanVar(value=False)
        self.output_dir_var = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))
        self.initial_exp_var = tk.StringVar(value="2002.0")
        self.min_exp_var = tk.StringVar(value="13.0")
        self.max_exp_var = tk.StringVar(value="4387.5")
        self.initial_gain_var = tk.StringVar(value="1.04")
        self.min_gain_var = tk.StringVar(value="1.04")
        self.max_gain_var = tk.StringVar(value="75.2")
        self.iterations_var = tk.StringVar(value="5")
        self.tolerance_var = tk.StringVar(value="0.10")
        self.settling_var = tk.StringVar(value="1")
        self.image_info_var = tk.StringVar(value="No image")

    def _build_ui(self) -> None:
        connection = ttk.LabelFrame(self, text="Connection — close Gpixel Cobra first")
        connection.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Label(connection, text="DLL").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(connection, textvariable=self.dll_var).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(connection, text="Browse", command=self._browse_dll).grid(row=0, column=2, padx=4)
        ttk.Label(connection, text="Device").grid(row=0, column=3, padx=4)
        ttk.Entry(connection, textvariable=self.device_var, width=6).grid(row=0, column=4)
        ttk.Label(connection, text="Timeout ms").grid(row=0, column=5, padx=4)
        ttk.Entry(connection, textvariable=self.timeout_var, width=9).grid(row=0, column=6)
        ttk.Button(connection, text="Connect", command=self._connect).grid(row=0, column=7, padx=4)
        ttk.Button(connection, text="Disconnect", command=self._disconnect).grid(row=0, column=8, padx=4)
        ttk.Label(connection, textvariable=self.connection_var).grid(row=1, column=0, columnspan=9,
                                                                      sticky="w", padx=4, pady=3)
        connection.columnconfigure(1, weight=1)

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=8, pady=4)
        self.mode_tab = ttk.Frame(notebook)
        self.sensor_tab = ttk.Frame(notebook)
        self.fpga_tab = ttk.Frame(notebook)
        self.image_tab = ttk.Frame(notebook)
        notebook.add(self.mode_tab, text="Mode")
        notebook.add(self.sensor_tab, text="Sensor")
        notebook.add(self.fpga_tab, text="FPGA")
        notebook.add(self.image_tab, text="Image / Auto")
        self._build_mode_tab()
        self._build_sensor_tab()
        self._build_fpga_tab()
        self._build_image_tab()

        log_frame = ttk.LabelFrame(self, text="Operation log")
        log_frame.pack(fill="both", padx=8, pady=(4, 8))
        self.log_text = tk.Text(log_frame, height=8, wrap="word", state="disabled")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def _build_mode_tab(self) -> None:
        box = ttk.LabelFrame(self.mode_tab, text="Canonical cold mode application")
        box.pack(fill="x", padx=12, pady=12)
        ttk.Button(box, text="Apply STD", command=lambda: self._apply_mode("std")).pack(side="left", padx=8, pady=12)
        ttk.Button(box, text="Apply HDR", command=lambda: self._apply_mode("hdr")).pack(side="left", padx=8, pady=12)
        ttk.Label(box, text="Runs power-off/on, FPGA table, sensor table, power sequence and training.",
                  wraplength=700).pack(side="left", padx=12)

    def _build_sensor_tab(self) -> None:
        single = ttk.LabelFrame(self.sensor_tab, text="Single register / range")
        single.pack(fill="x", padx=12, pady=(12, 6))
        for column, (label, variable, width) in enumerate((
                ("Address", self.sensor_addr_var, 12), ("Data", self.sensor_data_var, 12),
                ("Read length", self.sensor_len_var, 8))):
            ttk.Label(single, text=label).grid(row=0, column=column * 2, padx=4, pady=8)
            ttk.Entry(single, textvariable=variable, width=width).grid(row=0, column=column * 2 + 1, padx=4)
        ttk.Button(single, text="Write", command=self._sensor_write_one).grid(row=0, column=6, padx=8)
        ttk.Button(single, text="Read", command=self._sensor_read).grid(row=0, column=7, padx=8)

        file_box = ttk.LabelFrame(self.sensor_tab, text="Ordered ADDR(4)+DATA(2) .mem file")
        file_box.pack(fill="x", padx=12, pady=6)
        ttk.Entry(file_box, textvariable=self.sensor_file_var).pack(side="left", fill="x", expand=True, padx=6, pady=8)
        ttk.Button(file_box, text="Browse", command=self._browse_sensor_file).pack(side="left", padx=4)
        ttk.Checkbutton(file_box, text="Verify after write", variable=self.sensor_verify_var).pack(side="left", padx=4)
        ttk.Button(file_box, text="Write file", command=self._sensor_write_file).pack(side="left", padx=4)
        ttk.Button(file_box, text="Compare", command=self._sensor_compare_file).pack(side="left", padx=6)

        self.sensor_tree = ttk.Treeview(self.sensor_tab, columns=("addr", "expected", "actual", "result"),
                                        show="headings", height=15)
        for column, title, width in (("addr", "Address", 110), ("expected", "Expected", 110),
                                     ("actual", "Actual", 110), ("result", "Result", 120)):
            self.sensor_tree.heading(column, text=title)
            self.sensor_tree.column(column, width=width, anchor="center")
        self.sensor_tree.pack(fill="both", expand=True, padx=12, pady=(6, 12))
        self.sensor_tree.tag_configure("bad", foreground="#b00020")

    def _build_fpga_tab(self) -> None:
        box = ttk.LabelFrame(self.fpga_tab, text="Official FPGA register bus")
        box.pack(fill="x", padx=12, pady=12)
        ttk.Label(box, text="Address").grid(row=0, column=0, padx=5, pady=10)
        ttk.Entry(box, textvariable=self.fpga_addr_var, width=14).grid(row=0, column=1, padx=5)
        ttk.Label(box, text="16-bit value").grid(row=0, column=2, padx=5)
        ttk.Entry(box, textvariable=self.fpga_data_var, width=14).grid(row=0, column=3, padx=5)
        ttk.Checkbutton(box, text="Request write ACK", variable=self.fpga_ack_var).grid(row=0, column=4, padx=8)
        ttk.Button(box, text="Write FPGA", command=self._fpga_write).grid(row=0, column=5, padx=8)
        warning = ("Hardware FPGA readback is not exposed by the confirmed official protocol. "
                   "Therefore this GUI does not provide a fake FPGA Read/Compare button. "
                   "Sensor registers can be read and compared in the Sensor tab.")
        ttk.Label(self.fpga_tab, text=warning, foreground="#9a4d00", wraplength=850,
                  justify="left").pack(anchor="w", padx=18, pady=10)

    def _build_image_tab(self) -> None:
        outer = ttk.Frame(self.image_tab)
        outer.pack(fill="both", expand=True, padx=10, pady=10)
        controls = ttk.Frame(outer)
        controls.pack(side="left", fill="y", padx=(0, 10))
        preview = ttk.LabelFrame(outer, text="Captured image preview")
        preview.pack(side="right", fill="both", expand=True)

        output = ttk.LabelFrame(controls, text="Output")
        output.pack(fill="x", pady=(0, 8))
        ttk.Entry(output, textvariable=self.output_dir_var, width=42).pack(side="left", padx=5, pady=7)
        ttk.Button(output, text="Browse", command=self._browse_output_dir).pack(side="left", padx=4)
        ttk.Button(controls, text="Capture", command=self._capture, width=22).pack(fill="x", pady=3)

        auto = ttk.LabelFrame(controls, text="STD automatic exposure")
        auto.pack(fill="x", pady=8)
        ttk.Label(auto, text="Apply STD mode first; AUTO does not reinitialize the sensor.",
                  foreground="#9a4d00", wraplength=280).grid(row=0, column=0, columnspan=2,
                                                              sticky="w", padx=5, pady=5)
        fields = (
            ("Initial exposure (us)", self.initial_exp_var), ("Min exposure (us)", self.min_exp_var),
            ("Max exposure (us)", self.max_exp_var), ("Initial analog gain", self.initial_gain_var),
            ("Min analog gain", self.min_gain_var), ("Max analog gain", self.max_gain_var),
            ("Iterations (1..5)", self.iterations_var), ("Tolerance", self.tolerance_var),
            ("Settling frames", self.settling_var),
        )
        for row, (label, variable) in enumerate(fields):
            grid_row = row + 1
            ttk.Label(auto, text=label).grid(row=grid_row, column=0, sticky="w", padx=5, pady=3)
            ttk.Entry(auto, textvariable=variable, width=14).grid(row=grid_row, column=1, padx=5, pady=3)
        ttk.Button(auto, text="AUTO", command=self._auto, width=22).grid(row=len(fields) + 1, column=0,
                                                                          columnspan=2, padx=5, pady=10)

        ttk.Label(preview, textvariable=self.image_info_var, wraplength=580).pack(anchor="w", padx=8, pady=6)
        self.image_label = ttk.Label(preview, anchor="center")
        self.image_label.pack(fill="both", expand=True, padx=8, pady=8)

    def _browse_dll(self) -> None:
        path = filedialog.askopenfilename(filetypes=(("DLL", "*.dll"), ("All files", "*.*")))
        if path:
            self.dll_var.set(path)

    def _browse_sensor_file(self) -> None:
        path = filedialog.askopenfilename(filetypes=(("MEM", "*.mem"), ("All files", "*.*")))
        if path:
            self.sensor_file_var.set(path)

    def _browse_output_dir(self) -> None:
        path = filedialog.askdirectory(initialdir=self.output_dir_var.get())
        if path:
            self.output_dir_var.set(path)

    def _log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _task(self, label: str, work: Callable[[], object], done: Callable[[object], None] | None = None) -> None:
        if self.busy:
            messagebox.showwarning("Busy", "Another hardware operation is still running.")
            return
        self.busy = True
        self._log(f"START {label}")

        def worker() -> None:
            try:
                result = work()
            except Exception as exc:  # Report complete context in the GUI log.
                detail = traceback.format_exc()
                self.after(0, lambda: self._task_failed(label, exc, detail))
            else:
                self.after(0, lambda: self._task_done(label, result, done))

        threading.Thread(target=worker, daemon=True, name=f"cc2405-{label}").start()

    def _task_done(self, label: str, result: object, done: Callable[[object], None] | None) -> None:
        self.busy = False
        if done is not None:
            done(result)
        if isinstance(result, str):
            self._log(result)
        self._log(f"DONE {label}")

    def _task_failed(self, label: str, exc: Exception, detail: str) -> None:
        self.busy = False
        self._log(f"ERROR {label}: {exc}\n{detail}")
        messagebox.showerror(label, str(exc))

    def _connect(self) -> None:
        self._task("Connect", lambda: self.backend.connect(Path(self.dll_var.get()),
                                                             int(self.device_var.get()),
                                                             int(self.timeout_var.get())),
                   lambda value: self.connection_var.set(str(value)))

    def _disconnect(self) -> None:
        self._task("Disconnect", self.backend.disconnect,
                   lambda value: self.connection_var.set(str(value)))

    def _apply_mode(self, mode: str) -> None:
        if not messagebox.askokcancel("Apply mode", f"Apply full canonical {mode.upper()} configuration and training?"):
            return
        self._task(f"Apply {mode.upper()}", lambda: self.backend.apply_mode(mode))

    def _sensor_write_one(self) -> None:
        def work() -> str:
            address, value = parse_int(self.sensor_addr_var.get()), parse_int(self.sensor_data_var.get())
            if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFF:
                raise ValueError("sensor address must be 0..0xFFFF and data 0..0xFF")
            return self.backend.write_sensor_values([(address, value)], self.sensor_verify_var.get())
        self._task("Sensor write", work)

    def _sensor_read(self) -> None:
        try:
            address = parse_int(self.sensor_addr_var.get())
            length = parse_int(self.sensor_len_var.get())
        except ValueError as exc:
            messagebox.showerror("Sensor read", f"Invalid address or length: {exc}")
            return

        def done(result: object) -> None:
            values = list(result)  # type: ignore[arg-type]
            self._clear_tree()
            for offset, value in enumerate(values):
                self.sensor_tree.insert("", "end", values=(f"0x{address + offset:04X}", "—",
                                                            f"0x{value:02X}", "READ"))
            self._log("Sensor data: " + " ".join(f"{value:02X}" for value in values))
        self._task("Sensor read", lambda: self.backend.read_sensor_values(address, length), done)

    def _sensor_file_values(self) -> list[tuple[int, int]]:
        path = Path(self.sensor_file_var.get())
        if not path.is_file():
            raise FileNotFoundError(f"sensor .mem file not found: {path}")
        return parse_mem(path)

    def _sensor_write_file(self) -> None:
        self._task("Sensor file write", lambda: self.backend.write_sensor_values(
            self._sensor_file_values(), self.sensor_verify_var.get()))

    def _sensor_compare_file(self) -> None:
        def done(result: object) -> None:
            rows = list(result)  # type: ignore[arg-type]
            self._clear_tree()
            mismatches = 0
            for address, expected, actual, match in rows:
                if not match:
                    mismatches += 1
                self.sensor_tree.insert("", "end", values=(f"0x{address:04X}", f"0x{expected:02X}",
                                                            f"0x{actual:02X}", "MATCH" if match else "MISMATCH"),
                                        tags=() if match else ("bad",))
            self._log(f"Sensor compare: registers={len(rows)}, mismatches={mismatches}")
        self._task("Sensor compare", lambda: self.backend.compare_sensor_values(self._sensor_file_values()), done)

    def _clear_tree(self) -> None:
        self.sensor_tree.delete(*self.sensor_tree.get_children())

    def _fpga_write(self) -> None:
        self._task("FPGA write", lambda: self.backend.write_fpga_value(
            parse_int(self.fpga_addr_var.get()), parse_int(self.fpga_data_var.get()), self.fpga_ack_var.get()))

    def _capture(self) -> None:
        self._task("Capture", lambda: self.backend.capture(Path(self.output_dir_var.get())), self._show_capture)

    def _show_capture(self, result: object) -> None:
        capture = result.capture if isinstance(result, AutoResult) else result
        if not isinstance(capture, CaptureResult):
            return
        preview = display_image(capture.image, capture.info["pixel_width"])
        preview.thumbnail((650, 470), Image.Resampling.LANCZOS)
        self.preview_photo = ImageTk.PhotoImage(preview)
        self.image_label.configure(image=self.preview_photo)
        self.image_info_var.set(
            f"{capture.info['columns']}×{capture.info['rows']}, frames={capture.info['frames']}, "
            f"bits={capture.info['pixel_width']} | PNG: {capture.png_path}")
        self._log(f"Saved raw: {capture.raw_path}")
        self._log(f"Saved PNG: {capture.png_path}")

    def _auto(self) -> None:
        try:
            iterations = int(self.iterations_var.get())
            kwargs = dict(
                initial_exposure_us=float(self.initial_exp_var.get()),
                min_exposure_us=float(self.min_exp_var.get()),
                max_exposure_us=float(self.max_exp_var.get()),
                initial_gain=float(self.initial_gain_var.get()),
                min_gain=float(self.min_gain_var.get()),
                max_gain=float(self.max_gain_var.get()),
                iterations=iterations,
                tolerance=float(self.tolerance_var.get()),
                settling_frames=int(self.settling_var.get()),
            )
        except ValueError as exc:
            messagebox.showerror("AUTO", f"Invalid numeric parameter: {exc}")
            return
        if not 1 <= iterations <= 5:
            messagebox.showerror("AUTO", "Iterations must be 1..5")
            return

        def done(result: object) -> None:
            assert isinstance(result, AutoResult)
            for record in result.records:
                self._log("AE " + ", ".join(f"{key}={value}" for key, value in record.items()))
            self._log(f"AUTO converged={result.converged}")
            self._show_capture(result)
        self._task("AUTO exposure", lambda: self.backend.auto_expose(Path(self.output_dir_var.get()), **kwargs), done)

    def _close(self) -> None:
        if self.busy and not messagebox.askyesno("Exit", "A hardware operation is running. Exit anyway?"):
            return
        try:
            self.backend.disconnect()
        finally:
            self.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="CC2405 official-protocol GUI")
    parser.add_argument("--check", action="store_true", help="validate imports/helpers without opening a window")
    args = parser.parse_args()
    if args.check:
        test = np.array([[0, 4095]], dtype=np.uint16)
        preview = display_image(test, 12)
        assert preview.mode == "L" and preview.size == (2, 1)
        assert parse_int("0x10") == 16
        print("cc2405_control_gui check OK")
        return 0
    app = CC2405Gui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
