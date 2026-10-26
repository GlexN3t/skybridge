#!/usr/bin/env python3
"""SkyBridge ground station — browser GUI over the MAVLink telemetry stream.

Pure Python stdlib. Two jobs:

1. Speak MAVLink v2 over UDP to the drone (virtual_drone.py, ArduPilot
   SITL, or later the real vehicle), announcing ourselves with
   HEARTBEATs and decoding the incoming telemetry (mavlink_decode.py).
2. Serve the HUD page (ground_station.html) and stream the decoded
   state to the browser via Server-Sent Events — no websocket
   dependency, works with any modern browser.

   python3 tools/ground_station.py            # http://127.0.0.1:8080
   python3 tools/ground_station.py --http-port 9000 --drone-port 14550
"""
import argparse
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from mavlink_decode import (decode_attitude, decode_global_position,  # noqa: E402
                            decode_heartbeat, decode_sys_status,
                            extract_frames)
from mavlink_wire import build_frame  # noqa: E402

GCS_SYSID = 255
GCS_COMPID = 190
STALE_AFTER_S = 2.0


class TelemetryState:
    """Latest decoded telemetry, shared between the UDP thread and the
    HTTP handlers. Counters are racy-by-design (worst case a display
    glitch); dict swaps are atomic under the GIL."""

    def __init__(self):
        self.lock = threading.Lock()
        self.updated_at = None
        self.counts = {"attitude": 0, "position": 0, "sys_status": 0,
                       "heartbeat": 0, "crc_dropped": 0}
        self.rate_window = []  # rx timestamps for msg/s
        self.attitude = None
        self.position = None
        self.battery = None
        self.heartbeat = None

    def ingest(self, frames):
        now = time.monotonic()
        with self.lock:
            for f in frames:
                mid = f["msgid"]
                if mid == 30:
                    d = decode_attitude(f["payload"])
                    if d:
                        self.attitude = d
                        self.counts["attitude"] += 1
                elif mid == 33:
                    d = decode_global_position(f["payload"])
                    if d:
                        self.position = d
                        self.counts["position"] += 1
                elif mid == 1:
                    d = decode_sys_status(f["payload"])
                    if d:
                        self.battery = d
                        self.counts["sys_status"] += 1
                elif mid == 0:
                    d = decode_heartbeat(f["payload"])
                    if d:
                        self.heartbeat = d
                        self.counts["heartbeat"] += 1
                else:
                    continue
                self.rate_window.append(now)
            self.rate_window = [t for t in self.rate_window
                                if now - t < 1.0]
            if frames:
                self.updated_at = now

    def snapshot(self):
        with self.lock:
            age = (None if self.updated_at is None
                   else time.monotonic() - self.updated_at)
            return {
                "link_ok": age is not None and age < STALE_AFTER_S,
                "age_s": round(age, 2) if age is not None else None,
                "msg_per_s": len(self.rate_window),
                "counts": dict(self.counts),
                "attitude": self.attitude,
                "position": self.position,
                "battery": self.battery,
                "heartbeat": self.heartbeat,
                "server_time": time.strftime("%H:%M:%S"),
            }


class DroneLink(threading.Thread):
    """Announces the GCS over UDP so MAVLink streams start flowing."""

    def __init__(self, drone_host, drone_port, state):
        super().__init__(daemon=True)
        self.drone = (drone_host, drone_port)
        self.state = state
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))

    def _heartbeat(self):
        payload = bytes([0x03, 0, 0, 0, 0x06, 0x00, 0x01, 0x04, 3])
        frame = build_frame(0, payload, self.seq & 0xFF,
                            GCS_SYSID, GCS_COMPID)
        self.seq += 1
        self.sock.sendto(frame, self.drone)

    def run(self):
        self._heartbeat()
        next_announce = time.monotonic() + 1.0
        self.sock.settimeout(0.1)
        while True:
            if time.monotonic() >= next_announce:
                self._heartbeat()
                next_announce = time.monotonic() + 1.0
            try:
                data, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            frames, _ = extract_frames(data)
            self.state.ingest(frames)


HTML_PATH = Path(__file__).with_name("ground_station.html")


class Handler(BaseHTTPRequestHandler):
    state = None  # injected in main()

    def do_GET(self):
        if self.path == "/":
            body = HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    snap = self.state.snapshot()
                    self.wfile.write(f"data: {json.dumps(snap)}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                return  # browser tab closed
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass  # keep the console readable while streaming


def main():
    ap = argparse.ArgumentParser(description="SkyBridge ground station GUI")
    ap.add_argument("--http-host", default="127.0.0.1",
                    help="HTTP bind address (default loopback)")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--drone-host", default="127.0.0.1")
    ap.add_argument("--drone-port", type=int, default=14550)
    args = ap.parse_args()

    state = TelemetryState()
    Handler.state = state

    link = DroneLink(args.drone_host, args.drone_port, state)
    link.start()

    server = ThreadingHTTPServer((args.http_host, args.http_port), Handler)
    print(f"[gs] telemetry HUD: http://{args.http_host}:{args.http_port}")
    print(f"[gs] drone link:    UDP {args.drone_host}:{args.drone_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
