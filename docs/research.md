# Research log

Running log of protocol findings, measurements, and decisions.
Newest entries at the top.

## 2026-10-27 (2) — Ground station GUI

- Built a browser HUD (artificial horizon, pitch ladder, roll arc,
  heading strip, battery/position/vehicle panels) with **zero
  dependencies**: `ground_station.py` serves the page and pushes
  decoded telemetry over Server-Sent Events (stdlib
  `ThreadingHTTPServer`, 10 Hz); the canvas rendering is vanilla JS.
  Chose SSE over WebSockets deliberately — one-way telemetry needs no
  bidirectional channel, and SSE survives proxies and reconnects for
  free (EventSource auto-reconnects).
- Added `mavlink_decode.py`, a line-by-line Python mirror of the
  firmware decoders, plus a conformance test asserting both languages
  agree on every field — including the SYS_STATUS/relative_alt offsets
  that regressed earlier. The same wire bytes now decode identically
  in C (ESP32 path) and Python (GCS path).
- Verified in a real browser: LINK OK, 18 msg/s, 0 dropped frames,
  HUD attitude matching the simulator's sine profile (roll swinging
  ±11.5°), battery draining on schedule, GPS dead-reckoning from the
  ArduPilot home position. Two screenshots ~4 s apart confirm live
  animation, not a frozen frame.

## 2026-10-27 — Cross-language interop: Python drone ↔ C bridge

- Built the hardware-free end-to-end pipeline:
  `virtual_drone.py` (MAVLink v2 quadcopter simulator, pure Python) →
  UDP → `bridge_sim.c` (the firmware's parser + decoders compiled for
  the host). 4-second CI runs decode ~90 frames with **0 CRC errors**.
- Three wire-format bugs caught by the interop test, in order:
  1. Python struct format typo (`Ifffffff` — 7 floats; ATTITUDE is
     6 floats after the timestamp). Crash, caught instantly.
  2. **CRC chaining bug:** the Python encoder finalized the CRC-16/X-25
     (applied the 0xFFFF xorout) *between* accumulate calls, so the
     CRC_EXTRA byte was folded into an already-finalized checksum.
     Every frame failed verification in C — 105/105 CRC errors. Fix:
     separate raw accumulate from finalize, mirroring the C code's
     structure. Symmetry lesson: both implementations must follow the
     same accumulate→extra→xorout order, or nothing interoperates.
  3. **Field offset bugs in the decoders:** SYS_STATUS voltage decoded
     at byte 12 (that's the CPU load field — the bridge reported
     "battery=0.35 V", which was actually load=350‰); relative
     altitude at byte 20 (the vx field — "rel_alt=32 km"). Correct
     offsets verified against mavlink.io: voltage@14, current@16,
     remaining@18; relative_alt@16. Pinned permanently by round-trip
     unit tests (`test_decode_*_offsets`).
- Field layouts cross-checked against the official message reference
  (mavlink.io/en/messages/common.html): ATTITUDE 28 B, SYS_STATUS 31 B
  base, GLOBAL_POSITION_INT 28 B — all match the C decoders exactly.
- Debugging by semantic sanity (battery=0.35 V is physically
  impossible) located the off-by-N field immediately: the wrong value
  *identified itself* as the neighboring field (350 = load‰).
- Integration test (`tools/test_integration.sh`) now gates CI: random
  loopback port, asserts ATTITUDE/POSITION/SYS_STATUS decode, zero CRC
  errors, plausible home position.

## 2026-10-25 — Parser bring-up

- Implemented the MAVLink v2 framing state machine and frame builder
  from the protocol spec (mavlink.io/en/guide/serialization.html).
- **Bug found & fixed during testing:** initial implementation treated
  the header between STX and payload as 10 bytes; the correct count is
  9 (LEN, INCOMPAT, COMPAT, SEQ, SYSID, COMPID, MSGID×3). Symptom: the
  parser ate the first payload byte, so every frame "desynced" by one.
  Caught immediately by the round-trip unit test — the host test suite
  paid for itself on day one.
- CRC-16/X-25 verified against the catalogue check value:
  `"123456789" → 0x906E`.
- Confirmed the role of the per-message CRC_EXTRA byte: the sender
  appends it to the CRC input and the parser must too, or every valid
  frame fails verification. The table lives in `mav_crc_extra()`;
  unknown message IDs currently use 0, which means unknown messages
  are unreceivable without extending the table — an accepted
  limitation for the telemetry subset we speak.

## Test suite (tools/test_mavlink_parse.c)

1. CRC known-answer (X-25 check value)
2. HEARTBEAT build → parse round-trip, field equality
3. Corrupted payload byte → MAV_PARSE_ERROR_CRC
4. Garbage bytes between frames → BAD_STX reported, stream stays in sync
5. Zero-payload frame round-trip
6. Two back-to-back frames in one stream, no resync needed
