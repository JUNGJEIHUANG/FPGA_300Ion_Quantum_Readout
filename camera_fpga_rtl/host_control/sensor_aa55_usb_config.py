#!/usr/bin/env python3
"""K7-compatible GSENSE6504BSI register burst read/write over FX3 USB3.

Write flow: AA55 SENSOR packet loads the FPGA register BRAM, then AA55 FPGA
command 0x0004 launches SPI. Read flow uses command 0x0005 and receives the
official-style 0x84 result header followed by one low-byte value per USB word.
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.wintypes as wt
import os
from pathlib import Path
import struct
import sys
import time

from sensor_aa55_packet import (contiguous_runs, fpga_reg_command,
                                fpga_register_packet, load_mem,
                                sensor_load_packet)

DEFAULT_DEVICE_PATH = (r"\\?\USB#VID_04B4&PID_00F1#5&cc3f949&0&17"
                       r"#{ae18aa60-7f6a-11d4-97dd-00010229b959}")


class _SingleTransfer(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("SetupOrIso", ctypes.c_ubyte * 12), ("Reserved", ctypes.c_ubyte),
                ("ucEndpointAddress", ctypes.c_ubyte), ("NtStatus", ctypes.c_ulong),
                ("UsbdStatus", ctypes.c_ulong), ("IsoPacketOffset", ctypes.c_ulong),
                ("IsoPacketLength", ctypes.c_ulong), ("BufferOffset", ctypes.c_ulong),
                ("BufferLength", ctypes.c_ulong)]


class CyUsbSession:
    IOCTL_SEND_NON_EP0 = 0x00220024

    def __init__(self, device_path: str, ep_out: int, ep_in: int):
        self.ep_out, self.ep_in = ep_out, ep_in
        k = ctypes.windll.kernel32
        k.CreateFileW.restype = wt.HANDLE
        self.k = k
        self.handle = k.CreateFileW(device_path, 0xC0000000, 3, None, 3, 0x80, None)
        if self.handle == wt.HANDLE(-1).value:
            raise OSError(f"CreateFileW failed, Win32 error={k.GetLastError()}")

    def close(self) -> None:
        if self.handle not in (None, wt.HANDLE(-1).value):
            self.k.CloseHandle(self.handle)
            self.handle = None

    def _transfer(self, endpoint: int, payload: bytes | None, length: int) -> bytes:
        header_size = ctypes.sizeof(_SingleTransfer)
        total_size = header_size + length
        buf = (ctypes.c_ubyte * total_size)()
        st = _SingleTransfer.from_buffer(buf)
        st.ucEndpointAddress = endpoint & 0xFF
        st.BufferOffset = header_size
        st.BufferLength = length
        if payload:
            ctypes.memmove(ctypes.addressof(buf) + header_size, payload, length)
        returned = wt.DWORD(0)
        ok = self.k.DeviceIoControl(self.handle, self.IOCTL_SEND_NON_EP0,
                                    ctypes.byref(buf), total_size,
                                    ctypes.byref(buf), total_size,
                                    ctypes.byref(returned), None)
        if not ok or st.NtStatus != 0 or st.UsbdStatus != 0:
            raise OSError(f"CyUSB transfer failed ep=0x{endpoint:02X}: ok={ok}, "
                          f"Win32={self.k.GetLastError()}, NtStatus=0x{st.NtStatus:08X}, "
                          f"UsbdStatus=0x{st.UsbdStatus:08X}, length={st.BufferLength}/{length}")
        actual = int(st.BufferLength)
        if payload is not None and actual != length:
            raise OSError(f"short CyUSB write: {actual}/{length}")
        return bytes(buf[header_size:header_size + actual])

    def write(self, payload: bytes) -> int:
        self._transfer(self.ep_out, payload, len(payload))
        return len(payload)

    def read(self, length: int) -> bytes:
        return self._transfer(self.ep_in, None, length)


class DllSession:
    def __init__(self, path: Path, index: int, timeout_ms: int):
        if not path.is_file():
            raise FileNotFoundError(f"FX3 DLL not found: {path}")
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(path.parent))
        self.lib = ctypes.CDLL(str(path))
        self.timeout_ms = timeout_ms
        P = ctypes.c_void_p
        # The vendor ABI is deliberately asymmetric: write takes ``length``
        # by value, while read receives it by pointer and updates it with the
        # transferred byte count.  Passing a pointer to bulk_write makes the
        # DLL interpret an address as the transfer size, which times out.
        self.lib.gp_fx3_bulk_write.argtypes = [
            P, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint32, ctypes.c_uint32]
        self.lib.gp_fx3_bulk_write.restype = ctypes.c_int
        self.lib.gp_fx3_bulk_read.argtypes = [
            P, ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_uint32]
        self.lib.gp_fx3_bulk_read.restype = ctypes.c_int
        self.lib.gp_fx3_create.argtypes = [ctypes.POINTER(P)]
        self.lib.gp_fx3_create.restype = ctypes.c_int
        self.lib.gp_fx3_open.argtypes = [P, ctypes.c_int]
        self.lib.gp_fx3_open.restype = ctypes.c_int
        self.lib.gp_fx3_close.argtypes = [P]
        self.lib.gp_fx3_destroy.argtypes = [P]
        self.ctx = P()
        if self.lib.gp_fx3_create(ctypes.byref(self.ctx)) != 0:
            raise RuntimeError("gp_fx3_create failed")
        if self.lib.gp_fx3_open(self.ctx, index) != 0:
            self.close()
            raise RuntimeError(f"gp_fx3_open({index}) failed")

    def close(self) -> None:
        if getattr(self, "ctx", None) and self.ctx.value:
            self.lib.gp_fx3_close(self.ctx)
            self.lib.gp_fx3_destroy(self.ctx)
            self.ctx = ctypes.c_void_p()

    def _io(self, name: str, data_or_len: bytes | int) -> bytes | int:
        is_read = isinstance(data_or_len, int)
        size = data_or_len if is_read else len(data_or_len)
        buf = (ctypes.c_ubyte * size)()
        if not is_read:
            ctypes.memmove(buf, data_or_len, size)
        n = ctypes.c_uint32(size)
        length_arg = ctypes.byref(n) if is_read else n
        ret = getattr(self.lib, name)(self.ctx, buf, length_arg, self.timeout_ms)
        # gp_native_fx3.dll returns 5 for a completed bulk-IN transfer even
        # though it has already updated ``n`` and filled the caller's buffer.
        # Treat that read status as success only when bytes were actually
        # transferred; short/zero reads are still rejected by the caller.
        # On timeout this DLL leaves ``n`` at the requested size and the
        # zero-initialized buffer untouched, also with status 5.  Require
        # observable payload bytes so a timeout cannot be mistaken for a
        # successful all-zero transfer.
        read_complete = (is_read and ret == 5 and 0 < n.value <= size and
                         any(buf[:n.value]))
        if ret != 0 and not read_complete:
            raise RuntimeError(f"{name} failed: ret={ret}, length={n.value}/{size}")
        return bytes(buf[:n.value]) if is_read else int(n.value)

    def write(self, payload: bytes) -> int:
        return int(self._io("gp_fx3_bulk_write", payload))

    def read(self, length: int) -> bytes:
        return bytes(self._io("gp_fx3_bulk_read", length))


def parse_response(raw: bytes, code: int, length: int) -> list[int]:
    expected = (2 + length) * 4
    if len(raw) != expected:
        raise RuntimeError(f"short response: {len(raw)}/{expected} bytes")
    words = list(struct.unpack(f"<{2 + length}I", raw))
    if words[0] != 0x0000AA55:
        raise RuntimeError(f"bad ACK header 0x{words[0]:08X}")
    got_code, got_len = (words[1] >> 24) & 0xFF, words[1] & 0xFFFF
    if got_code != code or got_len != length:
        raise RuntimeError(f"bad ACK info code=0x{got_code:02X}, len={got_len}; "
                           f"expected code=0x{code:02X}, len={length}")
    return [word & 0xFF for word in words[2:]]


def main() -> int:
    p = argparse.ArgumentParser(description="K7-compatible SENSOR register USB read/write")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--mem", type=Path, help="24-bit MEM configuration file")
    src.add_argument("--reg", nargs=2, metavar=("ADDR", "DATA"), help="single write")
    src.add_argument("--read", nargs=2, metavar=("ADDR", "LEN"), help="continuous read")
    p.add_argument("--transport", choices=("cyusb", "dll"), default="cyusb")
    p.add_argument("--device-path", default=DEFAULT_DEVICE_PATH)
    p.add_argument("--dll", type=Path, default=Path(__file__).resolve().parent /
                   "third_party" / "cobra_dlls" / "gp_native_fx3.dll")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--ep-out", type=lambda x: int(x, 0), default=0x01)
    p.add_argument("--ep-in", type=lambda x: int(x, 0), default=0x81)
    p.add_argument("--timeout-ms", type=int, default=5000)
    p.add_argument("--output", type=Path, help="save read bytes")
    p.add_argument("--save-packet", type=Path, help="save concatenated OUT packets")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--no-auto-enable", action="store_true",
                   help="for --mem, skip PLL_EN and STREAM_EN writes")
    a = p.parse_args()

    packets: list[bytes] = []
    operations: list[tuple[str, bytes, int, int]] = []
    if a.read:
        addr, length = (int(x, 0) for x in a.read)
        # Official flow: first set FPGA registers reg_addr (1) and
        # reg_length (2), then issue RD_SENSOR_SPI (0x0005).  The former are
        # intentionally no-ACK writes, exactly as the official register bus.
        setup = fpga_register_packet([(0x0001, addr), (0x0002, length)])
        packets.append(setup)
        operations.append((f"set reg_addr=0x{addr:04X}, reg_length={length}",
                           setup, -1, 0))
        cmd = fpga_reg_command(False, addr, length)
        packets.append(cmd)
        operations.append((f"read 0x{addr:04X}+{length}", cmd, 0x84, length))
    else:
        if a.mem:
            regs = load_mem(a.mem)
        else:
            addr, data = (int(x, 0) for x in a.reg)
            if not 0 <= data <= 0xFF:
                p.error("DATA must be 0..0xFF")
            regs = [(addr, data)]
        load = sensor_load_packet(regs, ack_mode=0)
        packets.append(load)
        operations.append((f"load {len(regs)} BRAM values", load, -1, 0))
        for start, length in contiguous_runs(regs):
            cmd = fpga_reg_command(True, start, length)
            packets.append(cmd)
            operations.append((f"write 0x{start:04X}+{length}", cmd, 0x83, 0))
        if a.mem and not a.no_auto_enable:
            for label, reg, value, delay in (("PLL_EN", 0x03EA, 0xEA, 0.005),
                                             ("STREAM_EN", 0x0000, 0x01, 0.005)):
                operations.append((f"delay {delay}", b"", -2, 0))
                load_one = sensor_load_packet([(reg, value)], ack_mode=0)
                cmd_one = fpga_reg_command(True, reg, 1)
                packets.extend((load_one, cmd_one))
                operations.extend(((f"load {label}", load_one, -1, 0),
                                   (f"write {label}", cmd_one, 0x83, 0)))

    if a.save_packet:
        a.save_packet.write_bytes(b"".join(packets))
    print(f"operations={len(operations)} OUT_bytes={sum(map(len, packets))} "
          f"ep_out=0x{a.ep_out:02X} ep_in=0x{a.ep_in:02X}")
    if a.dry_run:
        for op, payload, code, length in operations:
            print(f"DRY {op}: tx={len(payload)} response={(2 + length) * 4 if code >= 0 else 0}")
        return 0

    session = (CyUsbSession(a.device_path, a.ep_out, a.ep_in) if a.transport == "cyusb"
               else DllSession(a.dll, a.device, a.timeout_ms))
    read_values: list[int] = []
    try:
        for op, payload, code, length in operations:
            if code == -2:
                time.sleep(float(op.split()[-1]))
                continue
            sent = session.write(payload)
            if sent != len(payload):
                raise RuntimeError(f"short write during {op}: {sent}/{len(payload)}")
            if code >= 0:
                values = parse_response(session.read((2 + length) * 4), code, length)
                read_values.extend(values)
            print(f"OK {op}")
    finally:
        session.close()

    if a.read:
        raw = bytes(read_values)
        print("read_data=" + raw.hex(" "))
        if a.output:
            a.output.write_bytes(raw)
            print(f"saved {len(raw)} bytes -> {a.output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        raise SystemExit(2)
