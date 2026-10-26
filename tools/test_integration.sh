#!/usr/bin/env bash
# End-to-end integration test: Python virtual drone <-> C firmware bridge.
#
# Starts tools/virtual_drone.py on a random loopback port, runs
# tools/bridge_sim (the firmware data path compiled for the host) against
# it, and asserts that real telemetry was decoded on the wire.
#
# Requires: gcc, python3. No network access, no downloads.
set -euo pipefail
cd "$(dirname "$0")/.."

PORT=$(( (RANDOM % 20000) + 30000 ))
LOG=build/integration.log

mkdir -p build

echo "[it] building..."
gcc -Wall -Wextra -Werror -O2 -I firmware/main \
    tools/bridge_sim.c firmware/main/mavlink_parse.c \
    firmware/main/mavlink_decode.c -o build/bridge_sim_it

echo "[it] starting virtual drone on UDP 127.0.0.1:$PORT..."
python3 tools/virtual_drone.py --port "$PORT" > build/integration_drone.log 2>&1 &
DRONE_PID=$!
trap 'kill "$DRONE_PID" 2>/dev/null || true' EXIT

sleep 1

echo "[it] running bridge_sim for 4 s..."
./build/bridge_sim_it 127.0.0.1 "$PORT" --duration 4 > "$LOG" 2>&1 || true

fail() { echo "[it] FAIL: $1"; cat "$LOG"; exit 1; }

grep -q "ATTITUDE" "$LOG"          || fail "no ATTITUDE decoded"
grep -q "POSITION" "$LOG"          || fail "no POSITION decoded"
grep -q "SYS_STATUS" "$LOG"        || fail "no SYS_STATUS decoded"
grep -q "crc_errors=0 " "$LOG"     || fail "CRC errors on the wire"
grep -q "lat=47.39" "$LOG"         || fail "latitude implausible"

echo "[it] PASS — telemetry decoded end-to-end with zero CRC errors"
head -4 "$LOG"
