#!/usr/bin/env python3
"""Conformance test: Python decoder must agree with the C decoder.

Builds frames with known payloads via mavlink_wire (the encoder used
on the wire), decodes them with mavlink_decode, and asserts exact
values — including the field offsets that once regressed
(SYS_STATUS voltage@14, GLOBAL_POSITION_INT relative_alt@16).
"""
import struct
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from mavlink_decode import (decode_attitude, decode_global_position,
                            decode_sys_status, extract_frames)
from mavlink_wire import build_frame

tests = 0


def frame_from_wire(msgid, payload, seq=0):
    frames, _ = extract_frames(build_frame(msgid, payload, seq))
    assert len(frames) == 1, f"expected 1 frame, got {len(frames)}"
    return frames[0]


# --- frame extraction: noise, resync, multi-frame ---
noise = bytes([0x00, 0xFF, 0x42])
f1 = build_frame(0, bytes(9), 1)
f2 = build_frame(30, bytes(28), 2)
frames, _ = extract_frames(noise + f1 + f2)
assert [fr["msgid"] for fr in frames] == [0, 30], "noise resync failed"
tests += 1

# corrupted frame must be dropped, following frames still parsed
bad = bytearray(build_frame(0, bytes(9), 5))
bad[13] ^= 0xFF
frames, _ = extract_frames(bytes(bad) + f1)
assert len(frames) == 1 and frames[0]["seq"] == 1, "CRC drop failed"
tests += 1

# --- SYS_STATUS offsets: load u16@12, voltage u16@14, current i16@16,
#     remaining i8@18 (the regression that showed as battery=0.35 V) ---
payload = bytearray(31)
struct.pack_into("<H", payload, 12, 350)    # load
struct.pack_into("<H", payload, 14, 12622)  # voltage mV
struct.pack_into("<h", payload, 16, 4200)   # current mA
payload[18] = 0xFD                          # remaining = -3 (int8)
bat = decode_sys_status(bytes(payload))
assert bat == {"voltage_mv": 12622, "current_ma": 4200,
               "battery_remaining_pct": -3}, bat
assert decode_sys_status(frame_from_wire(1, bytes(payload))["payload"]) == bat
tests += 1

# --- GLOBAL_POSITION_INT offsets: lat i32@4, lon i32@8, rel_alt i32@16
#     (the regression that showed as rel_alt=32 km) ---
payload = bytearray(28)
struct.pack_into("<I", payload, 0, 1000)
struct.pack_into("<ii", payload, 4, 473977418, 85455938)
struct.pack_into("<i", payload, 16, 50000)
pos = decode_global_position(bytes(payload))
assert pos["lat_e7"] == 473977418, pos
assert pos["lon_e7"] == 85455938, pos
assert pos["rel_alt_mm"] == 50000, pos
tests += 1

# --- ATTITUDE: u32 time then 6 floats (roll, pitch, yaw, ...) ---
payload = bytearray(28)
struct.pack_into("<I", payload, 0, 4242)
struct.pack_into("<fff", payload, 4, 0.2, -0.15, 1.5)
att = decode_attitude(bytes(payload))
assert abs(att["roll"] - 0.2) < 1e-6 and abs(att["pitch"] + 0.15) < 1e-6
assert abs(att["yaw"] - 1.5) < 1e-6, att
tests += 1

# --- short payloads rejected, not misdecoded ---
assert decode_attitude(bytes(27)) is None
assert decode_sys_status(bytes(30)) is None
assert decode_global_position(bytes(27)) is None
tests += 1

print(f"OK: {tests} python-decode test(s) passed")
