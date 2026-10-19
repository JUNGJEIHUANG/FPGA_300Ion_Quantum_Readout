#!/usr/bin/env python3
"""Build Gpixel FPGA-PC AA55 SENSOR packets for an FX3 bulk OUT endpoint.

This utility deliberately only builds the protocol payload; the FX3 endpoint
open/read/write operation remains board/firmware-specific and is not guessed.
Words are emitted little-endian, matching the 32-bit FX3 bus convention.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import struct


def load_mem(path: Path) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    for line_no, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        s = raw.split("//", 1)[0].strip()
        if not s:
            continue
        if len(s) != 6 or any(c not in "0123456789abcdefABCDEF" for c in s):
            raise ValueError(f"invalid 24-bit MEM word at {path}:{line_no}")
        out.append((int(s[:4], 16) & 0x7FFF, int(s[4:], 16) & 0xFF))
    return out


def packet(packet_type: int, words: list[int], ack_time: int = 1, ack_mode: int = 1) -> bytes:
    """Build one official CC2405 AA55 command.

    The fourth information byte is *ack time*, not ACK payload.  The official
    host always uses ``1`` (ACK after the FPGA completes the command); ``0``
    is documented but not implemented by the official firmware.
    """
    header = ((packet_type & 0xFF) << 16) | 0xAA55
    info = ((ack_time & 0xFF) << 24) | ((ack_mode & 0xFF) << 16) | len(words)
    all_words = [header, info, *words]
    all_words.append(sum(all_words) & 0xFFFFFFFF)
    return b"".join(struct.pack("<I", w) for w in all_words)


def sensor_load_packet(regs: list[tuple[int, int]], ack_mode: int = 0) -> bytes:
    """Build the K7 SENSOR-BRAM load packet (no SPI command by itself)."""
    words = [((addr & 0xFFFF) << 16) | (data & 0xFF) for addr, data in regs]
    return packet(1, words, ack_time=1, ack_mode=ack_mode)


def fpga_reg_command(write: bool, start_addr: int, length: int,
                     ack_mode: int = 1) -> bytes:
    """Build K7 FPGA command 0x0004 (write) or 0x0005 (read)."""
    if not 0 <= start_addr <= 0x3FFF:
        raise ValueError("K7 register BRAM address must be 0..0x3FFF")
    if not 1 <= length <= 0x4000 - start_addr:
        raise ValueError("register burst exceeds the K7 16K register BRAM")
    words = [
        (0x0001 << 16) | start_addr,
        (0x0002 << 16) | length,
        (0x0000 << 16) | (0x0004 if write else 0x0005),
    ]
    return packet(0, words, ack_time=1, ack_mode=ack_mode)


def fpga_register_packet(registers: list[tuple[int, int]], ack_mode: int = 0) -> bytes:
    """Write CC2405 FPGA control registers (no acknowledgement by default)."""
    words = [((addr & 0xFFFF) << 16) | (data & 0xFFFF)
             for addr, data in registers]
    return packet(0, words, ack_time=1, ack_mode=ack_mode)


def contiguous_runs(regs: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Return (start,length) runs without writing across unspecified gaps."""
    if not regs:
        return []
    runs: list[tuple[int, int]] = []
    start = prev = regs[0][0]
    for addr, _ in regs[1:]:
        if addr != prev + 1:
            runs.append((start, prev - start + 1))
            start = addr
        prev = addr
    runs.append((start, prev - start + 1))
    return runs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mem", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--ack-time", type=lambda x: int(x, 0), default=1)
    ap.add_argument("--ack-mode", type=lambda x: int(x, 0), default=1)
    args = ap.parse_args()
    regs = load_mem(args.mem)
    args.out.write_bytes(sensor_load_packet(regs, args.ack_mode))
    print(f"wrote {len(words)} SENSOR words ({(len(words)+3)*4} bytes) to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
