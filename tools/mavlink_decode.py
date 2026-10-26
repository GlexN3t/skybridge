"""Python mirror of firmware/main/mavlink_decode.c.

Decodes the payload of the telemetry messages we speak, plus a
buffer-based frame extractor. The ground-station GUI consumes this;
tools/test_python_decode.py asserts Python and C agree on every field.
"""
import struct

from mavlink_wire import STX, CRC_EXTRA, _crc_raw


class FrameError(Exception):
    pass


def extract_frames(buf: bytes):
    """Scan a UDP payload for complete MAVLink v2 frames.

    Returns (frames, consumed) where frames is a list of dicts with
    msgid, seq, sysid, compid, payload(bytes). Bytes before the first
    STX and any frame that fails its checksum are discarded — the same
    resync-on-noise behavior as the firmware parser.
    """
    frames = []
    i = 0
    n = len(buf)
    while i < n:
        if buf[i] != STX:
            i += 1
            continue
        if n - i < 12:  # STX + 9 header + 2 CRC, empty payload minimum
            break
        payload_len = buf[i + 1]
        total = 1 + 9 + payload_len + 2
        if n - i < total:
            break
        header = buf[i + 1:i + 10]
        payload = buf[i + 10:i + 10 + payload_len]
        wire_crc = buf[i + 10 + payload_len] | (buf[i + 11 + payload_len] << 8)
        msgid = header[6] | (header[7] << 8) | (header[8] << 16)

        crc = _crc_raw(header, 0xFFFF)
        crc = _crc_raw(payload, crc)
        crc = _crc_raw(bytes([CRC_EXTRA.get(msgid, 0)]), crc)
        if (crc ^ 0xFFFF) != wire_crc:
            i += 1  # resync like the C parser: treat as noise
            continue

        frames.append({
            "msgid": msgid,
            "seq": header[3],
            "sysid": header[4],
            "compid": header[5],
            "payload": payload,
        })
        i += total
    return frames, i


def decode_attitude(payload: bytes):
    """-> dict(rad) per firmware mav_decode_attitude (28-byte payload)."""
    if len(payload) < 28:
        return None
    vals = struct.unpack_from("<Iffffff", payload, 0)
    return {"time_boot_ms": vals[0], "roll": vals[1], "pitch": vals[2],
            "yaw": vals[3], "rollspeed": vals[4], "pitchspeed": vals[5],
            "yawspeed": vals[6]}


def decode_global_position(payload: bytes):
    """-> dict per mav_decode_global_position (28-byte payload)."""
    if len(payload) < 28:
        return None
    vals = struct.unpack_from("<IiiiihhhH", payload, 0)
    return {"time_boot_ms": vals[0], "lat_e7": vals[1], "lon_e7": vals[2],
            "alt_mm": vals[3], "rel_alt_mm": vals[4], "vx_cms": vals[5],
            "vy_cms": vals[6], "vz_cms": vals[7], "hdg_cdeg": vals[8]}


def decode_sys_status(payload: bytes):
    """-> dict per mav_decode_sys_status (31-byte payload)."""
    if len(payload) < 31:
        return None
    voltage_mv, current_ma = struct.unpack_from("<Hh", payload, 14)
    remaining = struct.unpack_from("<b", payload, 18)[0]
    return {"voltage_mv": voltage_mv, "current_ma": current_ma,
            "battery_remaining_pct": remaining}


def decode_heartbeat(payload: bytes):
    if len(payload) < 9:
        return None
    vals = struct.unpack_from("<IBBBBB", payload, 0)
    return {"custom_mode": vals[0], "mav_type": vals[1],
            "mav_autopilot": vals[2], "base_mode": vals[3],
            "system_status": vals[4], "mavlink_version": vals[5]}
