#!/usr/bin/env bash
# Start ArduPilot SITL (simulated quadcopter) accepting MAVLink
# connections from the ESP32 on UDP port 14550.
#
# Prerequisite: ArduPilot SITL installed. Official source:
#   https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
set -euo pipefail

command -v sim_vehicle.py >/dev/null 2>&1 || {
    echo "sim_vehicle.py not found — install ArduPilot SITL first."
    echo "See https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html"
    exit 1
}

exec sim_vehicle.py -v ArduCopter --out=udpin:0.0.0.0:14550 "$@"
