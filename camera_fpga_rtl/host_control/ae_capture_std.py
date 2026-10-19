#!/usr/bin/env python3
"""Capture-driven GSENSE6504BSI STD automatic exposure, at most five rounds.

Close Gpixel's official application before running this script. The script
configures an explicit initial exposure/gain, captures one official CC2405
frame per iteration, evaluates 12-bit P95 and saturation, and applies the next
hardware-realizable setting. Every image filename starts with T_.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import shutil
import sys

import numpy as np
from PIL import Image

from auto_exposure_std import (
    AEConfig, AEState, build_sensor_gain_sequence_from_setting,
    encode_exposure_registers, select_gain, supported_gain_settings, update_ae,
    _quantize_exposure,
)
from cc2405_official_usb import Fx3Session, capture_frame, sensor_read, sensor_write, write_fpga

BASE_DIR = Path(__file__).resolve().parent


def decode_first_frame(raw: bytes, info: dict[str, int]) -> np.ndarray:
    if info["bytes_per_pixel"] != 2 or info["pixel_width"] != 12:
        raise RuntimeError(
            f"expected 12-bit pixels in 16-bit containers, got {info['pixel_width']}-bit, "
            f"{info['bytes_per_pixel']} bytes/pixel")
    count = info["frames"] * info["rows"] * info["columns"]
    values = np.frombuffer(raw, dtype="<u2", count=count)
    stack = values.reshape(info["frames"], info["rows"], info["columns"])
    return (stack[0] & 0x0FFF).astype(np.uint16, copy=False)


def save_iteration(raw: bytes, image12: np.ndarray, output_dir: Path,
                   iteration: int) -> tuple[Path, Path]:
    raw_path = output_dir / f"T_AE_iter{iteration:02d}_raw.bin"
    png_path = output_dir / f"T_AE_iter{iteration:02d}.png"
    raw_path.write_bytes(raw)
    Image.fromarray((image12 << 4).astype(np.uint16), mode="I;16").save(png_path)
    return raw_path, png_path


def apply_gain(session: Fx3Session, setting) -> None:
    # A0/A1 are category-A registers. Read existing bytes so unrelated bits are
    # preserved, then use H<04> group hold to refresh gain synchronously.
    reg04 = sensor_read(session, 0x0004, 1)[0]
    reg_a0, reg_a1 = sensor_read(session, 0x00A0, 2)
    sequence = build_sensor_gain_sequence_from_setting(setting, reg04, reg_a0, reg_a1)
    sensor_write(session, list(sequence), verify=False)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--initial-exposure-us", type=float, required=True)
    p.add_argument("--min-exposure-us", type=float, required=True)
    p.add_argument("--max-exposure-us", type=float, required=True)
    p.add_argument("--initial-analog-gain", type=float, required=True)
    p.add_argument("--min-analog-gain", type=float, required=True)
    p.add_argument("--max-analog-gain", type=float, required=True)
    p.add_argument("--iterations", type=int, choices=range(1, 6), default=5,
                   metavar="1..5")
    p.add_argument("--target-tolerance", type=float, default=0.10,
                   help="stop when P95 is within this relative error of 2500 (default 0.10)")
    p.add_argument("--output-dir", type=Path,
                   default=BASE_DIR / "Test_images" / "T_AE_run")
    p.add_argument("--dll", type=Path,
                   default=BASE_DIR / "third_party" / "cobra_dlls" / "gp_native_fx3.dll")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--timeout-ms", type=int, default=30000)
    p.add_argument("--settling-frames", type=int, default=1,
                   help="discard frames after applying the initial setting (default 1)")
    args = p.parse_args()
    if not 0 < args.target_tolerance < 1:
        p.error("--target-tolerance must be in (0,1)")
    if args.settling_frames < 0:
        p.error("--settling-frames must be >= 0")

    cfg = AEConfig(
        min_exposure_us=args.min_exposure_us,
        max_exposure_us=args.max_exposure_us,
        min_analog_gain=args.min_analog_gain,
        max_analog_gain=args.max_analog_gain,
    )
    if args.max_exposure_us > cfg.max_external_exposure_us:
        print(f"warning: requested max exposure {args.max_exposure_us:.1f} us exceeds "
              f"fixed STD frame limit; clamped to {cfg.max_external_exposure_us:.1f} us")
    gains = supported_gain_settings(cfg.min_analog_gain, cfg.max_analog_gain)
    initial_gain = select_gain(gains, args.initial_analog_gain)
    initial_exposure = _quantize_exposure(args.initial_exposure_us, cfg)
    state = AEState(initial_exposure, initial_gain.analog_gain)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []
    final_png: Path | None = None
    converged = False

    session = Fx3Session(args.dll, args.device, args.timeout_ms)
    try:
        # Establish a known starting state; FPGA exposure registers are not
        # readable through the official register-bus abstraction.
        write_fpga(session, list(encode_exposure_registers(state.exposure_us, cfg.line_time_us)))
        apply_gain(session, initial_gain)

        # The DDR/capture pipeline can still contain a frame produced with the
        # preceding STD-mode settings.  Discard it so iteration 1 measures the
        # explicit AE starting state rather than stale exposure/gain.
        for settling_index in range(args.settling_frames):
            capture_frame(session, args.timeout_ms)
            print(f"discarded settling frame {settling_index + 1}/{args.settling_frames}")

        for iteration in range(1, args.iterations + 1):
            raw, info = capture_frame(session, args.timeout_ms)
            image12 = decode_first_frame(raw, info)
            raw_path, png_path = save_iteration(raw, image12, args.output_dir, iteration)
            final_png = png_path

            result = update_ae(image12, state, cfg)
            relative_error = abs(result.p95 - cfg.target_p95) / cfg.target_p95
            converged = (result.saturated_pixels <= cfg.saturation_pixel_threshold
                         and relative_error <= args.target_tolerance)
            records.append({
                "iteration": iteration,
                "p95": f"{result.p95:.3f}",
                "saturated_pixels": result.saturated_pixels,
                "exposure_us_used": f"{state.exposure_us:.6f}",
                "analog_gain_used": f"{state.analog_gain:.6f}",
                "next_exposure_us": f"{result.exposure_us:.6f}",
                "next_analog_gain": f"{result.analog_gain:.6f}",
                "reason": "target reached" if converged else result.reason,
                "raw_file": str(raw_path),
                "png_file": str(png_path),
            })
            print(f"iter={iteration} P95={result.p95:.1f} sat={result.saturated_pixels} "
                  f"used=({state.exposure_us:.1f} us, x{state.analog_gain:.2f}) "
                  f"next=({result.exposure_us:.1f} us, x{result.analog_gain:.2f}) "
                  f"{'CONVERGED' if converged else result.reason}")
            if converged:
                break

            if result.exposure_us != state.exposure_us:
                write_fpga(session, list(result.fpga_exposure_registers))
            if result.gain_setting.analog_gain != state.analog_gain:
                apply_gain(session, result.gain_setting)
            state = AEState(result.exposure_us, result.analog_gain)

            if iteration == args.iterations:
                # The iteration count limits parameter updates.  Apply update
                # number N, then acquire one verification frame without making
                # an (N+1)th update.
                raw, info = capture_frame(session, args.timeout_ms)
                image12 = decode_first_frame(raw, info)
                check_raw = args.output_dir / "T_AE_final_check_raw.bin"
                check_png = args.output_dir / "T_AE_final_check.png"
                check_raw.write_bytes(raw)
                Image.fromarray((image12 << 4).astype(np.uint16), mode="I;16").save(check_png)
                final_png = check_png
                check = update_ae(image12, state, cfg)
                relative_error = abs(check.p95 - cfg.target_p95) / cfg.target_p95
                converged = (check.saturated_pixels <= cfg.saturation_pixel_threshold
                             and relative_error <= args.target_tolerance)
                records.append({
                    "iteration": "final_check",
                    "p95": f"{check.p95:.3f}",
                    "saturated_pixels": check.saturated_pixels,
                    "exposure_us_used": f"{state.exposure_us:.6f}",
                    "analog_gain_used": f"{state.analog_gain:.6f}",
                    "next_exposure_us": f"{state.exposure_us:.6f}",
                    "next_analog_gain": f"{state.analog_gain:.6f}",
                    "reason": "verification: target reached" if converged else "verification: limit reached",
                    "raw_file": str(check_raw),
                    "png_file": str(check_png),
                })
                print(f"final_check P95={check.p95:.1f} sat={check.saturated_pixels} "
                      f"used=({state.exposure_us:.1f} us, x{state.analog_gain:.2f}) "
                      f"converged={converged}")
                break
    finally:
        session.close()

    csv_path = args.output_dir / "T_AE_summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=records[0].keys())
        writer.writeheader(); writer.writerows(records)
    if final_png is not None:
        final_path = args.output_dir / "T_AE_final.png"
        shutil.copy2(final_png, final_path)
        print(f"final image: {final_path}")
    print(f"summary: {csv_path}; converged={converged}")
    return 0 if converged else 3


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
