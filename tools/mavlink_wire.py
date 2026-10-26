#!/usr/bin/env python3
"""MAVLink v2 wire-format encoder (Python side of the interop pair).

Mirrors firmware/main/mavlink_parse.c: same frame layout, same
CRC-16/X-25 with per-message CRC_EXTRA. Used by tools/virtual_drone.py
to emit frames, and by the integration test to prove that a frame
built in Python parses identically in the C firmware parser.
"""
import struct

CRC_EXTRA = {
    0: 50,    # HEARTBEAT
    1: 124,   # SYS_STATUS
    30: 39,   # ATTITUDE
    33: 104,  # GLOBAL_POSITION_INT
}

STX = 0xFD


def crc16_x25(data: bytes) -> int:
    """CRC-16/X-25 (reflected 0x8408, init 0xFFFF, xorout 0xFFFF)."""
    return _crc_raw(data, 0xFFFF) ^ 0xFFFF


def _crc_raw(data: bytes, crc: int) -> int:
    """Accumulate into the CRC state WITHOUT the final XOR — chaining
    multiple accumulate calls must not finalize in between (the frame
    checksum covers header+payload+CRC_EXTRA in one continuous chain)."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def build_frame(msgid: int, payload: bytes, seq: int,
                sysid: int = 1, compid: int = 1) -> bytes:
    """Serialize one MAVLink v2 frame (firmware mav_frame_build twin)."""
    if len(payload) > 255:
        raise ValueError("payload too long")
    header = struct.pack(
        "<BBBBBB",
        len(payload),      # LEN
        0, 0,              # INCOMPAT / COMPAT flags
        seq & 0xFF,
        sysid,
        compid,
    ) + struct.pack("<I", msgid)[:3]

    crc = _crc_raw(header + payload, 0xFFFF)
    crc = _crc_raw(bytes([CRC_EXTRA.get(msgid, 0)]), crc)
    crc ^= 0xFFFF
    return bytes([STX]) + header + payload + struct.pack("<H", crc)
