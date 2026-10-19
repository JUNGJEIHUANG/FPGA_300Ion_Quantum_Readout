#!/usr/bin/env python3
"""Official-CC2405 USB control and capture utility.

This tool follows the protocol recovered from Gpixel's CC2405 0.2.3 software:
  * AA55 FPGA/sensor register packets;
  * sensor SPI commands 0x0004 (write) and 0x0005 (read);
  * command 0x0007 (FRAME_REQ) followed by the official 20-byte image header.

Safety: this program performs no write unless a write subcommand is explicitly
selected.  Capture only sends FRAME_REQ; it does not change sensor registers.
Close the official application before accessing the FX3 device.
"""
from __future__ import annotations

import argparse
import ctypes
import os
from pathlib import Path
import struct
import sys
import time
from typing import Iterable

try:
    import numpy as np
    from PIL import Image
except ImportError:  # Explicit error is raised only for --png.
    np = None
    Image = None

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_DLL = BASE_DIR / "third_party" / "cobra_dlls" / "gp_native_fx3.dll"
FRAME_REQUEST = 0x0007
WR_SENSOR_SPI = 0x0004
RD_SENSOR_SPI = 0x0005
ACK_WR_SENSOR_SPI = 0x83
ACK_RD_SENSOR_SPI = 0x84
ACK_FRAME_REQUEST = 0x86
SYNC_WORD = 0x0000AA55
FIRST_BLOCK_DWORDS = 65531
FIRST_PAYLOAD_BYTES = FIRST_BLOCK_DWORDS * 4  # 262124 pixel bytes
FIRST_TRANSFER_BYTES = 20 + FIRST_PAYLOAD_BYTES  # 262144 including frame header
BODY_BLOCK_BYTES = 65536 * 4


def packet(bank_sensor: bool, entries: Iterable[tuple[int, int]], *, ack_mode: int = 0,
           ack_time: int = 0) -> bytes:
    """Build an official AA55 packet; entries are (16-bit address, 16-bit data)."""
    entries = list(entries)
    if ack_mode not in (0, 1, 2) or ack_time not in (0, 1):
        raise ValueError("ack_mode must be 0..2 and ack_time must be 0 or 1")
    words = [SYNC_WORD | (int(bank_sensor) << 16),
             len(entries) | (ack_mode << 16) | (ack_time << 24)]
    # Official Python stores data first then address in the 32-bit little-endian DW.
    words.extend((data & 0xFFFF) | ((address & 0xFFFF) << 16)
                 for address, data in entries)
    words.append(sum(words) & 0xFFFFFFFF)
    return struct.pack(f"<{len(words)}I", *words)


def parse_command_reply(raw: bytes, expected_ack: int, expected_words: int = 0) -> list[int]:
    """Decode an official AA55 command reply (one low byte per returned DW)."""
    expected_bytes = (2 + expected_words) * 4
    if len(raw) != expected_bytes:
        raise RuntimeError(f"command reply length {len(raw)} != {expected_bytes}")
    words = struct.unpack(f"<{2 + expected_words}I", raw)
    if words[0] != SYNC_WORD:
        raise RuntimeError(f"unexpected reply sync 0x{words[0]:08X}")
    ack = (words[1] >> 24) & 0xFF
    count = words[1] & 0xFFFF
    if ack != expected_ack or count != expected_words:
        raise RuntimeError(f"unexpected reply ack=0x{ack:02X}, words={count}; "
                           f"expected ack=0x{expected_ack:02X}, words={expected_words}")
    return [word & 0xFF for word in words[2:]]


def parse_mem(path: Path) -> list[tuple[int, int]]:
    """Read an existing six-hex-digit ADDR+DATA .mem file without modifying it."""
    result: list[tuple[int, int]] = []
    for line_no, text in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        token = text.split("//", 1)[0].strip()
        if not token:
            continue
        if len(token) != 6 or any(c not in "0123456789abcdefABCDEF" for c in token):
            raise ValueError(f"{path}:{line_no}: expected six hex digits ADDR(4)+DATA(2)")
        result.append((int(token[:4], 16), int(token[4:], 16)))
    if not result:
        raise ValueError(f"{path}: contains no register entries")
    return result


def contiguous_runs(values: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Return contiguous address ranges; never write across an unspecified gap."""
    if not values:
        return []
    runs: list[tuple[int, int]] = []
    start = previous = values[0][0]
    for address, _ in values[1:]:
        if address != previous + 1:
            runs.append((start, previous - start + 1))
            start = address
        previous = address
    runs.append((start, previous - start + 1))
    return runs


def contiguous_value_runs(values: list[tuple[int, int]]) -> list[list[tuple[int, int]]]:
    """Split an ordered register program into consecutive-address runs.

    A sensor .mem program may intentionally revisit an address.  Loading the
    entire file into FPGA BRAM first would overwrite an earlier value before
    its SPI command executes, so each run must be loaded and committed in its
    original order.
    """
    if not values:
        return []
    runs = [[values[0]]]
    for value in values[1:]:
        if value[0] == runs[-1][-1][0] + 1:
            runs[-1].append(value)
        else:
            runs.append([value])
    return runs


class Fx3Session:
    """Minimal ctypes binding to Gpixel's gp_native_fx3 DLL."""
    def __init__(self, dll_path: Path, device_index: int, timeout_ms: int):
        if not dll_path.is_file():
            raise FileNotFoundError(f"FX3 DLL not found: {dll_path}")
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(dll_path.parent))
        self.lib = ctypes.CDLL(str(dll_path))
        P = ctypes.c_void_p
        self.lib.gp_fx3_create.argtypes = [ctypes.POINTER(P)]
        self.lib.gp_fx3_create.restype = ctypes.c_int
        self.lib.gp_fx3_open.argtypes = [P, ctypes.c_int]
        self.lib.gp_fx3_open.restype = ctypes.c_int
        self.lib.gp_fx3_close.argtypes = [P]
        self.lib.gp_fx3_close.restype = ctypes.c_int
        self.lib.gp_fx3_destroy.argtypes = [P]
        self.lib.gp_fx3_destroy.restype = ctypes.c_int
        self.lib.gp_fx3_bulk_write.argtypes = [P, ctypes.POINTER(ctypes.c_ubyte),
                                                ctypes.c_uint32, ctypes.c_uint32]
        self.lib.gp_fx3_bulk_write.restype = ctypes.c_int
        self.lib.gp_fx3_bulk_read.argtypes = [P, ctypes.POINTER(ctypes.c_ubyte),
                                               ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        self.lib.gp_fx3_bulk_read.restype = ctypes.c_int
        self.ctx = P()
        self.timeout_ms = timeout_ms
        if self.lib.gp_fx3_create(ctypes.byref(self.ctx)) != 0:
            raise RuntimeError("gp_fx3_create failed")
        if self.lib.gp_fx3_open(self.ctx, device_index) != 0:
            self.close()
            raise RuntimeError(f"gp_fx3_open({device_index}) failed; close the official application first")

    def close(self) -> None:
        if getattr(self, "ctx", None) and self.ctx.value:
            self.lib.gp_fx3_close(self.ctx)
            self.lib.gp_fx3_destroy(self.ctx)
            self.ctx = ctypes.c_void_p()

    def write(self, data: bytes) -> None:
        buffer = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        result = self.lib.gp_fx3_bulk_write(self.ctx, buffer, len(data), self.timeout_ms)
        if result != 0:
            raise RuntimeError(f"gp_fx3_bulk_write failed: {result}")

    def read(self, requested: int, timeout_ms: int | None = None) -> bytes:
        buffer = (ctypes.c_ubyte * requested)()
        length = ctypes.c_uint32(requested)
        result = self.lib.gp_fx3_bulk_read(self.ctx, buffer, ctypes.byref(length),
                                           self.timeout_ms if timeout_ms is None else timeout_ms)
        # This vendor DLL also returns status 5 for some completed Bulk-IN
        # transfers.  A timeout uses the same status while leaving the
        # zero-initialized buffer untouched, so accept 5 only when observable
        # payload was transferred.
        status5_complete = (result == 5 and 0 < length.value <= requested
                            and any(buffer[:length.value]))
        if result != 0 and not status5_complete:
            raise RuntimeError(f"gp_fx3_bulk_read failed: {result}, bytes={length.value}/{requested}")
        return bytes(buffer[:length.value])


def write_fpga(session: Fx3Session, values: list[tuple[int, int]], *, ack: bool = False) -> None:
    """Write FPGA registers.  Reads are not exposed by official hardware; FPGA bus is write-only."""
    session.write(packet(False, values, ack_mode=int(ack), ack_time=0))
    if ack:
        parse_command_reply(session.read(8), expected_ack=0x80, expected_words=0)


def sensor_write(session: Fx3Session, values: list[tuple[int, int]], verify: bool = False) -> None:
    # Loading sensor values and executing each contiguous SPI range are distinct official operations.
    # Keep ordered runs separate so repeated-address timing programs remain intact.
    for run in contiguous_value_runs(values):
        address, length = run[0][0], len(run)
        session.write(packet(True, run, ack_mode=0, ack_time=0))
        session.write(packet(False, [(0x0001, address), (0x0002, length), (0x0000, WR_SENSOR_SPI)],
                             ack_mode=1, ack_time=0))
        parse_command_reply(session.read(8), ACK_WR_SENSOR_SPI)
    if verify:
        # Verify the final state only; a repeated address is intentionally
        # represented by its last programmed value.
        final_values = dict(values)
        ordered_final = sorted(final_values.items())
        for address, length in contiguous_runs(ordered_final):
            got = sensor_read(session, address, length)
            wanted = [final_values[reg] for reg in range(address, address + length)]
            if got != wanted:
                raise RuntimeError(f"verify failed at 0x{address:04X}: got={got!r}, want={wanted!r}")


def sensor_read(session: Fx3Session, address: int, length: int) -> list[int]:
    if not 1 <= length <= 0x3FFF:
        raise ValueError("sensor read length must be 1..16383 bytes")
    # Official software writes reg_addr/reg_length then invokes RD_SENSOR_SPI.
    write_fpga(session, [(0x0001, address), (0x0002, length)])
    session.write(packet(False, [(0x0000, RD_SENSOR_SPI)], ack_mode=1, ack_time=1))
    return parse_command_reply(session.read((2 + length) * 4), ACK_RD_SENSOR_SPI, length)


def capture_frame(session: Fx3Session, timeout_ms: int) -> tuple[bytes, dict[str, int]]:
    """Request exactly one official frame and return pixels plus validated header fields."""
    # Cobra calls send(FRAME_REQ, ack_content=1), which maps to ack_time=1.
    # The normal USB response starts with the 20-byte frame header (whose byte
    # 7 is ACK_FRAME_REQUEST) and may contain up to 65531 payload DWs.  The FX3
    # DLL is allowed to return a short USB transfer, so a 20-byte header-only
    # first read is valid and the remaining pixels are collected below.
    session.write(packet(False, [(0x0000, FRAME_REQUEST)], ack_mode=1, ack_time=1))
    first = session.read(FIRST_TRANSFER_BYTES, timeout_ms)

    # Be tolerant of firmware variants that emit a standalone 8-byte FRAME_REQ
    # ACK before the actual frame-header transfer.
    if len(first) == 8:
        parse_command_reply(first, ACK_FRAME_REQUEST, 0)
        first = session.read(FIRST_TRANSFER_BYTES, timeout_ms)
    if len(first) < 20:
        raise RuntimeError(
            f"short frame header: {len(first)}/20 bytes; data={first.hex(' ')}")
    if int.from_bytes(first[:4], "little") != SYNC_WORD:
        raise RuntimeError(f"invalid frame-header sync: {first[:4].hex(' ')}")
    if first[7] != ACK_FRAME_REQUEST:
        raise RuntimeError(
            f"invalid frame-header ACK 0x{first[7]:02X}; expected 0x{ACK_FRAME_REQUEST:02X}")
    columns = int.from_bytes(first[8:10], "little")
    rows = int.from_bytes(first[10:12], "little")
    frames = int.from_bytes(first[12:14], "little")
    pixel_width = first[16]
    if not rows or not columns or not frames or not 1 <= pixel_width <= 16:
        raise RuntimeError(f"invalid frame header: rows={rows}, columns={columns}, frames={frames}, bits={pixel_width}")
    bytes_per_pixel = (pixel_width + 7) // 8
    total = rows * columns * frames * bytes_per_pixel
    pixels = bytearray(first[20:20 + total])
    while len(pixels) < total:
        wanted = min(BODY_BLOCK_BYTES, total - len(pixels))
        block = session.read(wanted, timeout_ms)
        if not block:
            raise RuntimeError(f"empty pixel body read at {len(pixels)}/{total}")
        pixels.extend(block)
    return bytes(pixels), {"columns": columns, "rows": rows, "frames": frames,
                           "pixel_width": pixel_width, "bytes_per_pixel": bytes_per_pixel}


def save_png(raw: bytes, info: dict[str, int], out_path: Path) -> None:
    if np is None or Image is None:
        raise RuntimeError("--png requires numpy and Pillow")
    if info["frames"] != 1 or info["bytes_per_pixel"] != 2:
        raise RuntimeError("PNG export currently supports one 16-bit-container frame only; raw .bin was saved")
    image = np.frombuffer(raw, dtype="<u2").reshape(info["rows"], info["columns"])
    # Pixel width is commonly 12; retain actual bits by left-aligning to 16-bit PNG.
    shift = max(0, 16 - info["pixel_width"])
    Image.fromarray((image << shift).astype(np.uint16), mode="I;16").save(out_path)


def add_transport(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dll", type=Path, default=DEFAULT_DLL)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--dry-run", action="store_true", help="print packets but do not open FX3")


def parse_int(value: str) -> int:
    return int(value, 0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("sensor-write", help="load and commit sensor register values")
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--mem", type=Path, help="six-hex-digit ADDR+DATA .mem file")
    group.add_argument("--reg", nargs=2, metavar=("ADDR", "DATA"), help="one sensor byte register")
    p.add_argument("--verify", action="store_true", help="read back every written contiguous range")
    add_transport(p)
    p = sub.add_parser("sensor-read", help="read byte registers through official RD_SENSOR_SPI")
    p.add_argument("address", type=parse_int); p.add_argument("length", type=parse_int); p.add_argument("--out", type=Path)
    add_transport(p)
    p = sub.add_parser("fpga-write", help="write one official 16-bit FPGA register")
    p.add_argument("address", type=parse_int); p.add_argument("data", type=parse_int); p.add_argument("--ack", action="store_true")
    add_transport(p)
    p = sub.add_parser("capture", help="send FRAME_REQ then receive one official image frame")
    p.add_argument("--out", type=Path, default=BASE_DIR / "Test_images" / "CC2405_frame.bin")
    p.add_argument("--png", action="store_true", help="also write CC2405_frame.png beside the raw image")
    add_transport(p)
    args = parser.parse_args()

    if args.command == "sensor-write":
        values = parse_mem(args.mem) if args.mem else [(parse_int(args.reg[0]), parse_int(args.reg[1]))]
        if any(not (0 <= a <= 0xFFFF and 0 <= d <= 0xFF) for a, d in values):
            parser.error("sensor address must be 0..0xffff and data 0..0xff")
        description = f"sensor-write registers={len(values)}, ranges={contiguous_runs(values)}"
    elif args.command == "sensor-read":
        description = f"sensor-read address=0x{args.address:04X}, length={args.length}"
    elif args.command == "fpga-write":
        if not 0 <= args.address <= 0xFFFF or not 0 <= args.data <= 0xFFFF:
            parser.error("FPGA address/data must be 0..0xffff")
        description = f"fpga-write 0x{args.address:04X}=0x{args.data:04X}"
    else:
        description = "capture FRAME_REQ=0x0007; no sensor/FPGA configuration writes"
    print(description)
    if args.dry_run:
        return 0

    session = Fx3Session(args.dll, args.device, args.timeout_ms)
    try:
        if args.command == "sensor-write":
            sensor_write(session, values, args.verify)
            print("OK sensor-write")
        elif args.command == "sensor-read":
            data = bytes(sensor_read(session, args.address, args.length))
            print("read_data=" + data.hex(" "))
            if args.out:
                args.out.write_bytes(data)
                print(f"saved {len(data)} bytes -> {args.out}")
        elif args.command == "fpga-write":
            write_fpga(session, [(args.address, args.data)], args.ack)
            print("OK fpga-write")
        else:
            raw, info = capture_frame(session, args.timeout_ms)
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_bytes(raw)
            print(f"frame={info}; saved {len(raw)} bytes -> {args.out}")
            if args.png:
                png = args.out.with_name("T_" + args.out.stem.removeprefix("T_") + ".png")
                save_png(raw, info, png)
                print(f"saved PNG -> {png}")
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
