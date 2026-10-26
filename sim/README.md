# Simulator (hardware-free development)

ArduPilot SITL (Software In The Loop) simulates a complete quadcopter
on your laptop, speaking MAVLink over UDP exactly like a real vehicle.
The ESP32 connects to it over WiFi, so all firmware development and
testing happens before any drone exists.

## Setup

1. Install ArduPilot SITL (see `tools/setup-toolchain.sh`, which lists
   the official sources — review before running anything).
2. Start the simulator:

   ```sh
   sim/start-sitl.sh
   ```

   This runs `sim_vehicle.py -v ArduCopter --out=udpin:0.0.0.0:14550`,
   i.e. it accepts MAVLink connections from any host on the LAN at
   UDP port 14550 — that endpoint is what the ESP32 firmware dials.

## Network model

- Laptop (running SITL): your WiFi access point / hotspot
- ESP32: joins that WiFi as a station, sends MAVLink to
  `<laptop-ip>:14550` (configure via `Kconfig` once net_link.c lands)
