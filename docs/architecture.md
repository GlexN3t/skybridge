# Architecture

## Components

**firmware (ESP32, ESP-IDF 5.x / FreeRTOS)**

- `mavlink_parse.c` — MAVLink v2 wire-format core: incremental framing
  state machine (one byte per call, no frame buffering), CRC-16/X-25
  verification with per-message CRC_EXTRA, and a frame builder for the
  messages we transmit. No dynamic allocation.
- (phase 1) `net_link.c` — WiFi station mode + UDP socket; feeds
  received bytes straight into the parser (zero-copy path).
- (phase 2) `dashboard.c` — HTTP + WebSocket server exposing the
  latest decoded telemetry as JSON.

**sim (laptop)**

- ArduPilot SITL — a full simulated quadcopter. Runs on the host,
  speaks MAVLink over UDP exactly like a real vehicle, so the firmware
  is developed and tested before any hardware exists.

## Design decisions

| Decision | Rationale |
|---|---|
| Hand-rolled parser instead of mavgen headers | Portfolio/research value: understand and document the wire format; ~200 auditable lines vs. a generated ~10k-line header set; no vendor tooling in the build. Revisit if the message set grows beyond ~20 types. |
| Byte-at-a-time parser API | Works identically over UART, UDP, or file streams; no intermediate buffer; trivially unit-testable on the host with gcc. |
| Host-side unit tests before hardware | The protocol core is the risky part and needs no chip to verify. CI runs it on every push. |
| CRC_EXTRA check table in firmware | The v2 checksum intentionally fails if parser and sender disagree on the message set — turning a silent desync into a loud error. |
| ESP-IDF instead of Arduino | Native FreeRTOS task/priority control, no Arduino abstraction layer, matches what employers expect for embedded roles. |

## MAVLink v2 frame (reference)

```
offset  size  field
0       1     STX = 0xFD
1       1     LEN           payload length (0-255)
2       1     INCOMPAT      e.g. bit 0 = message is signed
3       1     COMPAT
4       1     SEQ           sender's packet counter
5       1     SYSID         sender system id
6       1     COMPID        sender component id
7       3     MSGID         little-endian, 24-bit
10      LEN   PAYLOAD       little-endian struct
10+LEN  2     CHECKSUM      CRC-16/X-25 over bytes 1..9+LEN
                            with the message's CRC_EXTRA appended
```
