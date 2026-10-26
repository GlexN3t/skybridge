#!/usr/bin/env bash
# SkyBridge toolchain setup — RUN MANUALLY, after reviewing.
#
# This script documents every download this project needs. Nothing is
# installed unless you run it yourself. Each source is the official
# upstream; verify before running.
set -euo pipefail

echo "== 1. ESP-IDF toolchain (Espressif official, ~1-2 GB) =="
echo "   Source: https://github.com/espressif/esp-idf (v5.2.2)"
echo "   git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf"
echo "   ~/esp/esp-idf/install.sh esp32"
echo "   . ~/esp/esp-idf/export.sh   # add to your shell profile"
echo
echo "== 2. ArduPilot SITL (simulator, for hardware-free dev) =="
echo "   Source: https://github.com/ArduPilot/ardupilot"
echo "   https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html"
echo "   (also available as pip package: pip install pymavlink MAVProxy)"
echo
echo "== Nothing above runs automatically. Read, then copy the lines you want. =="
