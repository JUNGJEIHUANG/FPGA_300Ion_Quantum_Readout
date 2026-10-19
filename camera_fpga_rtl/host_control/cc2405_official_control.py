#!/usr/bin/env python3
"""CC2405 control/capture utility for the official Gpixel ``top.bit``.

This tool implements the AA55 command protocol recovered from the matching
Gpixel Cobra application.  It deliberately does not use the self-developed
bitstream's TCL flag handshake or 0xFFFFFFFF frame marker.

Functions:
  * configure/read GSENSE6504 sensor registers through the FPGA SPI bridge;
  * write CC2405 FPGA registers;
  * request and receive an official-format image from the FPGA DDR path.

Register writes are never performed implicitly.  Use ``--dry-run`` to inspect
the exact OUT packets without opening the FX3 device.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import struct
import sys
import time
from typing import Iterable

import numpy as np
from PIL import Image

from sensor_aa55_packet import (
    fpga_reg_command,
    fpga_register_packet,
    packet as aa55_packet,
    sensor_load_packet,
)
from sensor_aa55_usb_config import DllSession


FRAME_REQ = 0x0007
ACK_FRAME_REQ = 0x86
FIRST_FRAME_DWORDS = 65_531
NEXT_FRAME_DWORDS = 65_536
CANONICAL_CONFIG_DIR = Path(__file__).resolve().parent / "Gpixel_reg_GT_config"


@dataclass(frozen=True)
class FrameHeader:
    columns: int
    rows: int
    frames: int
    pixel_width: int
    ack_code: int

    @property
    def bytes_per_pixel(self) -> int:
        return (self.pixel_width + 7) // 8

    @property
    def image_bytes(self) -> int:
        return self.frames * self.rows * self.columns * self.bytes_per_pixel


def frame_request_packet() -> bytes:
    """Build the official FPGA FRAME_REQ command.

    Cobra calls ``send(SystemCommand.FRAME_REQ, ack_content=1)``.  In the
    extracted protocol, ``ack_content`` is passed as ``ack_time``; therefore
    this command uses normal ACK mode and completion-time ACK.
    """
    return fpga_register_packet([(0x0000, FRAME_REQ)], ack_mode=1)


def system_command_packet(command: int, *, ack_time: int = 0) -> bytes:
    """Build one official FPGA system command at main-register address zero."""
    word = (0x0000 << 16) | (command & 0xFFFF)
    return aa55_packet(0, [word], ack_time=ack_time, ack_mode=1)


def fpga_write_packet(registers: list[tuple[int, int]]) -> bytes:
    words = [((address & 0xFFFF) << 16) | (value & 0xFFFF)
             for address, value in registers]
    return aa55_packet(0, words, ack_time=0, ack_mode=0)


def sensor_bram_packet(registers: list[tuple[int, int]]) -> bytes:
    words = [((address & 0xFFFF) << 16) | (value & 0xFF)
             for address, value in registers]
    return aa55_packet(1, words, ack_time=0, ack_mode=0)


def load_canonical_mode(mode: str) -> tuple[list[tuple[int, int]], list[tuple[int, int]]]:
    """Load the ordered FPGA/Sensor programs exported from the matching EXE."""
    mode = mode.upper()
    fpga_path = CANONICAL_CONFIG_DIR / f"CC2405_{mode}_FPGA_embedded.csv"
    sensor_path = CANONICAL_CONFIG_DIR / f"CC2405_{mode}_SENSOR_embedded.csv"
    if not fpga_path.is_file() or not sensor_path.is_file():
        raise FileNotFoundError(
            f"canonical {mode} CSV files not found in {CANONICAL_CONFIG_DIR}"
        )

    fpga: list[tuple[int, int]] = []
    with fpga_path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = sorted(csv.DictReader(handle), key=lambda row: int(row["sequence"]))
        for row in rows:
            address = int(row["address_hex"], 0)
            width = int(row["width_bits"])
            value = int(row["value_dec"])
            if width % 16 or width < 16:
                raise ValueError(f"unsupported FPGA width {width} for {row['name']}")
            for word_index in range(width // 16):
                fpga.append((address + word_index, (value >> (16 * word_index)) & 0xFFFF))

    sensor: list[tuple[int, int]] = []
    with sensor_path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = sorted(csv.DictReader(handle), key=lambda row: int(row["sequence"]))
        for row in rows:
            sensor.append((int(row["address_hex"], 0), int(row["value_dec"])))
    if not fpga or not sensor:
        raise ValueError(f"empty canonical {mode} configuration")
    return fpga, sensor


def parse_frame_header(block: bytes) -> tuple[FrameHeader, bytes]:
    """Parse the official 20-byte frame header and return its pixel payload."""
    if len(block) < 20:
        raise RuntimeError(f"short first frame block: {len(block)}/20 bytes")
    if int.from_bytes(block[0:4], "little") != 0x0000AA55:
        raise RuntimeError(f"bad frame sync: {block[0:4].hex(' ')}")
    ack_code = block[7]
    if ack_code == 0xFF:
        raise RuntimeError("FPGA returned command error (ACK 0xFF)")
    if ack_code != ACK_FRAME_REQ:
        raise RuntimeError(
            f"unexpected frame ACK 0x{ack_code:02X}; expected 0x{ACK_FRAME_REQ:02X}"
        )
    header = FrameHeader(
        columns=int.from_bytes(block[8:10], "little"),
        rows=int.from_bytes(block[10:12], "little"),
        frames=int.from_bytes(block[12:14], "little"),
        pixel_width=block[16],
        ack_code=ack_code,
    )
    if not (header.columns and header.rows and header.frames):
        raise RuntimeError(f"invalid zero-sized frame header: {header}")
    if not 1 <= header.pixel_width <= 16:
        raise RuntimeError(f"unsupported pixel width: {header.pixel_width}")
    return header, block[20:]


def parse_int(value: str) -> int:
    return int(value, 0)


def require_range(name: str, value: int, low: int, high: int) -> int:
    if not low <= value <= high:
        raise ValueError(f"{name} must be 0x{low:X}..0x{high:X}, got 0x{value:X}")
    return value


def save_capture(raw: bytes, header: FrameHeader, output_dir: Path, prefix: str) -> None:
    if not prefix.startswith("T_"):
        raise ValueError("output prefix must start with T_")
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / f"{prefix}_raw_{header.columns}x{header.rows}x{header.frames}.bin"
    raw_path.write_bytes(raw)
    print(f"saved raw: {raw_path}")

    count = header.frames * header.rows * header.columns
    if header.bytes_per_pixel == 1:
        pixels = np.frombuffer(raw, dtype=np.uint8, count=count).reshape(
            header.frames, header.rows, header.columns
        )
        for index, frame in enumerate(pixels):
            path = output_dir / f"{prefix}_frame{index:04d}_{header.columns}x{header.rows}.png"
            Image.fromarray(frame, mode="L").save(path)
            print(f"saved image: {path}")
        return

    pixels16 = np.frombuffer(raw, dtype="<u2", count=count).reshape(
        header.frames, header.rows, header.columns
    )
    shift = max(0, 16 - header.pixel_width)
    for index, frame in enumerate(pixels16):
        # Preserve the sensor code values in a standard 16-bit grayscale PNG.
        display16 = (frame.astype(np.uint16) << shift).astype(np.uint16)
        path = output_dir / f"{prefix}_frame{index:04d}_{header.columns}x{header.rows}.png"
        Image.fromarray(display16, mode="I;16").save(path)
        print(f"saved image: {path}")


def run_operations(session: DllSession, operations: Iterable[tuple[str, bytes, int, int]]) -> list[int]:
    values: list[int] = []
    for label, payload, ack_code, response_values in operations:
        sent = session.write(payload)
        if sent != len(payload):
            raise RuntimeError(f"short write during {label}: {sent}/{len(payload)}")
        if ack_code >= 0:
            response_size = (2 + response_values) * 4
            response = session.read(response_size)
            if len(response) != response_size:
                raise RuntimeError(f"short response during {label}: {len(response)}/{response_size}")
            words = struct.unpack(f"<{2 + response_values}I", response)
            if words[0] != 0x0000AA55:
                raise RuntimeError(f"bad ACK sync during {label}: 0x{words[0]:08X}")
            got_code = (words[1] >> 24) & 0xFF
            got_len = words[1] & 0xFFFF
            if got_code != ack_code or got_len != response_values:
                raise RuntimeError(
                    f"bad ACK during {label}: code=0x{got_code:02X}, len={got_len}; "
                    f"expected code=0x{ack_code:02X}, len={response_values}"
                )
            values.extend(word & 0xFF for word in words[2:])
        print(f"OK {label}")
    return values


def build_sensor_write_operations(regs: list[tuple[int, int]]) -> list[tuple[str, bytes, int, int]]:
    if not regs:
        raise ValueError("no sensor registers supplied")
    operations: list[tuple[str, bytes, int, int]] = []
    # Match Cobra exactly: each ordered (address,value) item is loaded, then
    # committed with its own WR_SENSOR_SPI command.  This is intentionally not
    # optimized because repeated indirect writes are timing/ordering sensitive.
    for index, (start, value) in enumerate(regs):
        load = sensor_bram_packet([(start, value)])
        operations.append(
            (f"load sensor item {index}: 0x{start:04X}=0x{value:02X}", load, -1, 0)
        )
        operations.append(
            (f"set sensor SPI range 0x{start:04X}+1",
             fpga_write_packet([(0x0001, start), (0x0002, 1)]), -1, 0)
        )
        operations.append(
            (f"sensor SPI write 0x{start:04X}+1", system_command_packet(0x0004), 0x83, 0)
        )
    return operations


def build_mode_operations(mode: str) -> list[tuple[str, bytes, int, int]]:
    """Reproduce the official cold mode-application sequence from canonical CSV."""
    fpga, sensor = load_canonical_mode(mode)
    operations: list[tuple[str, bytes, int, int]] = [
        ("POWER_OFF", system_command_packet(0x0002), 0x81, 0),
        ("POWER_ON", system_command_packet(0x0001), 0x80, 0),
    ]
    for index, (address, value) in enumerate(fpga):
        operations.append(
            (f"FPGA mode item {index}: 0x{address:04X}=0x{value:04X}",
             fpga_write_packet([(address, value)]), -1, 0)
        )

    # Cobra's _change_register_main_dummy raises frame_linetime from the table's
    # seed value 1 to 676 for these shared STD/HDR LP timing values.
    operations.append(
        ("FPGA derived frame_linetime=676", fpga_write_packet([(0x0100, 676), (0x0101, 0)]), -1, 0)
    )
    operations.extend(build_sensor_write_operations(sensor))
    operations.extend([
        ("POWER_ON_SEQUENCE", system_command_packet(0x1006), 0xAF, 0),
        ("TRAINING", system_command_packet(0x0006, ack_time=1), 0x85, 112),
    ])
    return operations


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Official Gpixel CC2405 control and image capture")
    parser.add_argument(
        "--dll",
        type=Path,
        default=Path(__file__).resolve().parent / "third_party" / "cobra_dlls" / "gp_native_fx3.dll",
    )
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--timeout-ms", type=int, default=10_000)
    parser.add_argument("--dry-run", action="store_true")
    sub = parser.add_subparsers(dest="action", required=True)

    sensor_write = sub.add_parser("sensor-write", help="write one sensor register")
    sensor_write.add_argument("address", type=parse_int)
    sensor_write.add_argument("data", type=parse_int)

    sensor_read = sub.add_parser("sensor-read", help="read contiguous sensor registers")
    sensor_read.add_argument("address", type=parse_int)
    sensor_read.add_argument("length", type=parse_int)
    sensor_read.add_argument("--output", type=Path)

    fpga_write = sub.add_parser("fpga-write", help="write one FPGA register")
    fpga_write.add_argument("address", type=parse_int)
    fpga_write.add_argument("data", type=parse_int)

    mode_apply = sub.add_parser("mode-apply", help="apply an embedded STD, HDR, or derived test-mode CSV sequence")
    mode_apply.add_argument(
        "mode",
        choices=(
            "std", "hdr", "std_gain1p04", "std_gain1p05",
            "hdr_gain1p04", "hdr_gain1p05", "hdr_gain1p04_1p05",
            "high_sensitive",
        ),
    )

    capture = sub.add_parser("capture", help="request and receive an official-format image")
    capture.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "Test_images")
    capture.add_argument("--prefix", default="CC2405_official")
    capture.add_argument("--frames", type=int, default=1,
                         help="number of frames; written to FPGA frame_num and Sensor burst_frm_num")
    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    operations: list[tuple[str, bytes, int, int]] = []

    if args.action == "sensor-write":
        address = require_range("sensor address", args.address, 0, 0x3FFF)
        data = require_range("sensor data", args.data, 0, 0xFF)
        operations = build_sensor_write_operations([(address, data)])
    elif args.action == "sensor-read":
        address = require_range("sensor address", args.address, 0, 0x3FFF)
        length = require_range("sensor read length", args.length, 1, 0x4000 - address)
        setup = fpga_write_packet([(0x0001, address), (0x0002, length)])
        operations = [
            (f"set sensor read range 0x{address:04X}+{length}", setup, -1, 0),
            (f"sensor SPI read 0x{address:04X}+{length}",
             system_command_packet(0x0005, ack_time=1), 0x84, length),
        ]
    elif args.action == "fpga-write":
        address = require_range("FPGA address", args.address, 0, 0xFFFF)
        data = require_range("FPGA data", args.data, 0, 0xFFFF)
        operations = [(f"FPGA write 0x{address:04X}=0x{data:04X}",
                       fpga_write_packet([(address, data)]), -1, 0)]
    elif args.action == "mode-apply":
        operations = build_mode_operations(args.mode)
    elif args.action == "capture":
        frames = require_range("frame count", args.frames, 1, 0xFFFF)
        capture_setup = [
            (f"FPGA capture frame_num={frames}",
             fpga_write_packet([(0x0003, frames)]), -1, 0),
        ]
        capture_setup.extend(build_sensor_write_operations([
            (0x0020, frames & 0xFF),
            (0x0021, (frames >> 8) & 0xFF),
        ]))
        packet = frame_request_packet()
        if args.dry_run:
            for label, payload, ack, count in capture_setup:
                print(
                    f"DRY {label}: tx={len(payload)} ack="
                    f"{'none' if ack < 0 else f'0x{ack:02X}/{count}'} packet={payload.hex(' ')}"
                )
            print(f"DRY FRAME_REQ tx={len(packet)}: {packet.hex(' ')}")
            return 0
        session = DllSession(args.dll, args.device, args.timeout_ms)
        try:
            run_operations(session, capture_setup)
            trigger_call_ns = time.perf_counter_ns()
            sent = session.write(packet)
            trigger_write_return_ns = time.perf_counter_ns()
            if sent != len(packet):
                raise RuntimeError(f"short FRAME_REQ write: {sent}/{len(packet)}")
            first_size = FIRST_FRAME_DWORDS * 4 + 20
            first = session.read(first_size)
            # The official bitstream may concatenate the 20-byte frame header
            # and the first image block in one bulk-IN transaction.  The self
            # HDL intentionally ends the command-response transaction after
            # the header and starts image streaming as the next transaction.
            # Both are valid as long as a complete frame header was returned.
            if len(first) < 20:
                raise RuntimeError(f"short frame header transfer: {len(first)}/20")
            header, first_pixels = parse_frame_header(first)
            print(
                f"frame header: {header.columns}x{header.rows}, frames={header.frames}, "
                f"pixel_width={header.pixel_width}, image_bytes={header.image_bytes}"
            )
            image = bytearray(first_pixels[: header.image_bytes])
            block_size = NEXT_FRAME_DWORDS * 4
            while len(image) < header.image_bytes:
                remaining = header.image_bytes - len(image)
                block = session.read(block_size)
                # Cobra always submits a full 256 KiB USB read, including for
                # the final image remainder.  The DLL reports the actual byte
                # count, so a short final transfer is valid (20 bytes for one
                # 2176x2048x12-bit frame); only an undersized non-final block
                # is an error.
                expected = min(block_size, remaining)
                if len(block) < expected:
                    raise RuntimeError(
                        f"short frame body transfer: {len(block)}/{expected} "
                        f"(requested {block_size})"
                    )
                image.extend(block[:remaining])
            image_complete_ns = time.perf_counter_ns()
            pc_timing_lines = [
                f"trigger_write_call_ms={(trigger_write_return_ns - trigger_call_ns) / 1e6:.6f}",
                f"trigger_call_to_complete_ms={(image_complete_ns - trigger_call_ns) / 1e6:.6f}",
                f"trigger_write_return_to_complete_ms={(image_complete_ns - trigger_write_return_ns) / 1e6:.6f}",
                f"received_image_bytes={len(image)}",
                "clock=time.perf_counter_ns",
                "complete_boundary=last required image byte returned by gp_native_fx3 bulk-IN read",
            ]
            args.output_dir.mkdir(parents=True, exist_ok=True)
            pc_timing_path = args.output_dir / f"{args.prefix}_pc_timing.txt"
            pc_timing_path.write_text("\n".join(pc_timing_lines) + "\n", encoding="utf-8")
            print("PC timing: " + ", ".join(pc_timing_lines[:3]))
            print(f"saved PC timing: {pc_timing_path}")
            save_capture(bytes(image), header, args.output_dir, args.prefix)
        finally:
            session.close()
        return 0

    if args.dry_run:
        for label, payload, ack, count in operations:
            print(
                f"DRY {label}: tx={len(payload)} ack="
                f"{'none' if ack < 0 else f'0x{ack:02X}/{count}'} packet={payload.hex(' ')}"
            )
        return 0

    session = DllSession(args.dll, args.device, args.timeout_ms)
    try:
        if args.action == "mode-apply":
            # Official mode code waits after programming and before training.
            training = operations.pop()
            power_sequence = operations.pop()
            values = run_operations(session, operations)
            run_operations(session, [power_sequence])
            time.sleep(0.3 if args.mode == "std" else 1.0)
            training_values = run_operations(session, [training])
            failed = [training_values[i:i + 7] for i in range(0, len(training_values), 7)
                      if len(training_values[i:i + 7]) == 7 and not training_values[i + 6]]
            print(f"training_channels={len(training_values) // 7} failed={len(failed)}")
            if failed:
                raise RuntimeError(f"training failed: {failed}")
        else:
            values = run_operations(session, operations)
    finally:
        session.close()
    if args.action == "sensor-read":
        raw = bytes(values)
        print("sensor data: " + raw.hex(" "))
        if args.output:
            args.output.write_bytes(raw)
            print(f"saved: {args.output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
