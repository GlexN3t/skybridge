#!/usr/bin/env python3
"""Virtual drone — a MAVLink v2 quadcopter simulator in pure Python.

Listens on a UDP port; the first station that sends us a HEARTBEAT
becomes our telemetry subscriber, and we stream sensor data to it at
realistic rates, exactly like an ArduPilot vehicle streams to a GCS:

  HEARTBEAT           1 Hz
  SYS_STATUS          2 Hz   (battery voltage/remaining, slowly draining)
  ATTITUDE           10 Hz   (sinusoidal roll/pitch, slow yaw rotation)
  GLOBAL_POSITION_INT 5 Hz   (dead-reckoned from attitude/airspeed)

Flight profile is deterministic (time-based), so two test runs against
the same build produce comparable output.

No external dependencies — this is a stand-in for ArduPilot SITL that
lets the whole stack be developed and CI-tested without installing
anything. sim/start-sitl.sh swaps this for the real simulator later.
"""
import argparse
import math
import socket
import select
import struct
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from mavlink_wire import build_frame  # noqa: E402

MAV_TYPE_QUADROTOR = 2
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_STATE_ACTIVE = 4
MAV_SYS_STATUS_SENSOR_3D_GYRO = 1 << 0
MAV_SYS_STATUS_SENSOR_3D_ACCEL = 1 << 1
MAV_SYS_STATUS_SENSOR_3D_MAG = 1 << 2
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 1 << 8
MAV_SYS_STATUS_SENSOR_GPS = 1 << 32  # sensor-present bitmask is 32-bit wide

BASE_LAT = 47.397742  # ArduPilot SITL default home position
BASE_LON = 8.545594
BASE_ALT_M = 488.0
REL_ALT_M = 50.0
GROUND_SPEED_MPS = 5.0


def encode_heartbeat(seq: int) -> bytes:
    payload = struct.pack(
        "<IBBBBB",
        3,  # custom_mode: ArduCopter flight mode 3 = AltHold
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        MAV_STATE_ACTIVE,
        3,  # mavlink_version
    )
    return build_frame(0, payload, seq)


def encode_sys_status(seq: int, voltage_mv: int, remaining_pct: int) -> bytes:
    sensors = (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL
               | MAV_SYS_STATUS_SENSOR_3D_MAG
               | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)
    payload = struct.pack(
        "<IIIHHHbHHHHHH",
        sensors & 0xFFFFFFFF,   # sensors present (GPS is beyond bit 31)
        sensors & 0xFFFFFFFF,   # enabled
        sensors & 0xFFFFFFFF,   # healthy
        350,                    # cpu load %
        voltage_mv,
        4200,                   # current draw mA
        remaining_pct,
        0, 0,                   # comm drop rate, comm errors
        0, 0, 0, 0,             # error counts
    )
    return build_frame(1, payload, seq)


def encode_attitude(seq: int, t: float) -> bytes:
    payload = struct.pack(
        "<Iffffff",
        int(t * 1000) & 0xFFFFFFFF,       # time_boot_ms
        0.20 * math.sin(t * 0.9),          # roll rad
        0.15 * math.cos(t * 0.7),          # pitch rad
        (t * 0.1) % (2 * math.pi),         # yaw rad
        0.05 * math.sin(t * 1.3),          # rollspeed
        0.04 * math.sin(t * 1.1),          # pitchspeed
        0.1,                               # yawspeed
    )
    return build_frame(30, payload, seq)


def encode_global_position(seq: int, t: float) -> bytes:
    heading_rad = (t * 0.1) % (2 * math.pi)
    dist_m = GROUND_SPEED_MPS * t
    # local NED dead-reckoning -> degrees
    lat = BASE_LAT + (dist_m * math.cos(heading_rad)) / 111_320.0
    dlon_m = dist_m * math.sin(heading_rad)
    lon = BASE_LON + dlon_m / (111_320.0 * math.cos(math.radians(BASE_LAT)))
    payload = struct.pack(
        "<IiiiihhhH",
        int(t * 1000) & 0xFFFFFFFF,
        int(lat * 1e7),
        int(lon * 1e7),
        int((BASE_ALT_M + REL_ALT_M) * 1000),   # alt MSL mm
        int(REL_ALT_M * 1000),                  # relative alt mm
        int(GROUND_SPEED_MPS * math.sin(heading_rad) * 100),  # vx cm/s
        int(GROUND_SPEED_MPS * math.cos(heading_rad) * 100),  # vy cm/s
        0,                                      # vz
        int(math.degrees(heading_rad) * 100) & 0xFFFF,        # hdg cdeg
    )
    return build_frame(33, payload, seq)


class VirtualDrone:
    RATES = [  # (message name, Hz, encoder)
        ("HEARTBEAT", 1, lambda t, s: encode_heartbeat(s)),
        ("SYS_STATUS", 2, lambda t, s: encode_sys_status(
            s, 12600 - int(t * 10), max(5, 100 - int(t / 60 * 100 * 0.2)))),
        ("ATTITUDE", 10, lambda t, s: encode_attitude(s, t)),
        ("GLOBAL_POSITION_INT", 5, lambda t, s: encode_global_position(s, t)),
    ]

    def __init__(self, port: int, host: str = "127.0.0.1"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.subscriber = None
        self.seq = 0
        self.sent = 0

    def poll(self, timeout: float) -> None:
        r, _, _ = select.select([self.sock], [], [], timeout)
        if r:
            data, addr = self.sock.recvfrom(2048)
            # Any packet from a new source (we only check it looks like a
            # MAVLink v2 HEARTBEAT) registers it as the telemetry target.
            if len(data) >= 12 and data[0] == 0xFD:
                if self.subscriber != addr:
                    print(f"[drone] telemetry subscriber: {addr[0]}:{addr[1]}",
                          flush=True)
                self.subscriber = addr

    def tick(self, t: float) -> int:
        n = 0
        for _, hz, encode in self.RATES:
            if int(t * hz) != int((t - 0.05) * hz):  # rate gate on 50 ms tick
                frame = encode(t, self.seq)
                self.seq = (self.seq + 1) & 0xFF
                if self.subscriber:
                    self.sock.sendto(frame, self.subscriber)
                    n += 1
        self.sent += n
        return n


def main() -> None:
    ap = argparse.ArgumentParser(description="MAVLink v2 virtual drone")
    ap.add_argument("--port", type=int, default=14550,
                    help="UDP port to listen on (default 14550)")
    ap.add_argument("--host", default="127.0.0.1",
                    help="interface to bind (default loopback; pass 0.0.0.0 "
                         "only when the ESP32 must reach you over WiFi)")
    args = ap.parse_args()

    drone = VirtualDrone(args.port, args.host)
    print(f"[drone] listening on UDP {args.host}:{args.port} — waiting for a "
          f"MAVLink heartbeat...", flush=True)

    start = time.monotonic()
    last = start
    while True:
        now = time.monotonic()
        if now - last >= 0.05:  # 20 Hz internal tick, per-rate gates above
            drone.tick(now - start)
            last = now
        drone.poll(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
