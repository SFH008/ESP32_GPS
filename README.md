# ESP32-S3 NMEA 2000 GPS Node

A standalone NMEA 2000 GPS node built on the ESP32-S3. Reads position data from a u-blox M10 GPS module and injects it onto a Raymarine SeaTalkNG / NMEA 2000 bus at 10 Hz, with secondary MQTT publishing and a live web dashboard for field debugging.

## Features

- Transmits four PGNs onto the N2K bus:
  - **129025** — Position, Rapid Update (10 Hz)
  - **129026** — COG & SOG, Rapid Update (10 Hz)
  - **129029** — GNSS Position Data (1 Hz)
  - **126992** — System Time (1 Hz)
- u-blox M10 configured for UBX binary, 10 Hz updates, maritime dynamic model
- Full NMEA 2000 address claiming — works alongside other GPS sources (e.g. Vesper AIS) on the same bus
- WiFi credentials configured via captive portal (WiFiManager) — no hardcoded secrets in firmware
- OTA firmware updates over WiFi
- MQTT publishing — JSON position payload for Node-RED / InfluxDB / Grafana, independent of the N2K path
- Live web dashboard with auto-refreshing map — `http://<node-ip>/`
- Powered directly from the STNG/N2K bus via buck converter — single cable installation

## Hardware

| Component | Part |
|---|---|
| Microcontroller | Sparkle IoT XH-S3E N16R8 (ESP32-S3-WROOM-1, 16MB flash, 8MB PSRAM) |
| GPS module | HGLRC M100 Pro (u-blox M10, multi-constellation, 10 Hz) |
| CAN transceiver | SN65HVD230 breakout |
| Bus connection | Raymarine SeaTalkNG drop cable (STNG) |
| Power | Pololu D24V10F5 buck converter (12V bus → 5V) |

## Wiring

### STNG Drop Cable → SN65HVD230

| STNG wire | SN65HVD230 pin |
|---|---|
| Net-S (CAN-H) | CANH |
| Net-C (CAN-L) | CANL |
| Shield/GND | GND |
| +12V | → Buck converter input |

> **Rs pin on SN65HVD230 must be tied to GND** for normal speed mode.

### SN65HVD230 → ESP32-S3

| SN65HVD230 | ESP32-S3 GPIO |
|---|---|
| TXD | GPIO4 |
| RXD | GPIO5 |
| VCC | 3.3V |
| GND | GND |

### HGLRC M100 Pro → ESP32-S3

JST-GH 1.25mm 6-pin connector. Only 4 wires used (compass SDA/SCL not connected). Confirmed pinout for this module — wire colors are NOT standardized across cable batches, always confirm against the module's own silkscreen labels:

| M100 Pro pin | Wire (this build) | ESP32-S3 |
|---|---|---|
| 5V | Red | 5V (VBUS or external buck converter) |
| GND | Black | GND |
| TX | Yellow | GPIO16 |
| RX | Green | GPIO17 |

> The M10 UART is 3.3V logic — connect directly to the ESP32-S3, no level shifter needed. Power must be 5V, not 3.3V.
>
> **Confirmed factory baud rate: 115200**, not the commonly assumed 38400 u-blox default. This module ships pre-configured for UBX binary output at 115200, likely inherited from its drone flight-controller heritage. Verified by raw UART capture during bring-up.

## Pin Summary

| Function | GPIO |
|---|---|
| TWAI TX (→ SN65HVD230 TXD) | GPIO4 |
| TWAI RX (← SN65HVD230 RXD) | GPIO5 |
| GPS UART RX (← M100 Pro TX) | GPIO16 |
| GPS UART TX (→ M100 Pro RX) | GPIO17 |

## Software

Built with [PlatformIO](https://platformio.org/) using the [pioarduino](https://github.com/pioarduino/platform-espressif32) platform fork, pinned to release `54.03.21` (Arduino core 3.2.1 / ESP-IDF 5.4.2). The `stable` tag is avoided — a later toolchain update dropped a compiler flag the NMEA2000 library depends on (`-mdisable-hardware-atomics`), breaking the build.

### Dependencies

- [ttlappalainen/NMEA2000-library](https://registry.platformio.org/libraries/ttlappalainen/NMEA2000-library)
- [sergei/NMEA2000_esp32_twai](https://github.com/sergei/NMEA2000_esp32_twai) — TWAI driver (not `ttlappalainen/NMEA2000_esp32`, which targets MCP2515/SPI)
- [sparkfun/SparkFun u-blox GNSS v3](https://registry.platformio.org/libraries/sparkfun/SparkFun%20u-blox%20GNSS%20v3) — use the `SFE_UBLOX_GNSS_SERIAL` class for UART (the plain `SFE_UBLOX_GNSS` class is I2C-only)
- [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager) — captive portal WiFi config
- [knolleary/PubSubClient](https://github.com/knolleary/pubsubclient) — MQTT
- [bblanchon/ArduinoJson](https://arduinojson.org/) — JSON payload building
- `WebServer.h` — bundled with the ESP32 Arduino core, no separate install needed

### Configuration

`include/Config.h` holds all pins, intervals, and N2K identity. No WiFi credentials are stored in source — see [WiFi Setup](#wifi-setup) below.

Key values to review before flashing:

```cpp
#define N2K_UNIQUE_NUMBER   2002   // Must be unique across all DIY N2K nodes on your bus
#define MQTT_BROKER         "192.168.2.120"
#define MQTT_TOPIC_GPS       "gps-n2k/position"
```

### Building and Flashing

**First flash (USB, required once)** — ensure the OTA lines in `platformio.ini` are commented out:

```ini
;upload_protocol = espota
;upload_port = gps-n2k.local
```

```bash
# Linux permissions for serial access (one-time, then re-login)
sudo usermod -aG dialout $USER

pio run -t upload --upload-port /dev/ttyACM0
```

**Subsequent updates — OTA over WiFi:**

```ini
upload_protocol = espota
upload_port = gps-n2k.local
```

```bash
pio run -t upload
# or, if mDNS doesn't resolve from your build host:
pio run -t upload --upload-port <node-ip>
```

### WiFi Setup

No credentials are hardcoded. On first boot (or after a reset), the node broadcasts an open access point:

1. Connect a phone or laptop to the AP **`GPS-N2K-Setup`**
2. A captive portal should open automatically (or browse to `192.168.4.1`)
3. Select **Configure WiFi**, choose your network, enter the password, save
4. The node reboots and connects automatically from then on

If no credentials are entered within 60 seconds, the node continues without WiFi — **the N2K bus and GPS are never blocked waiting on WiFi configuration.** OTA and MQTT simply remain unavailable until WiFi is configured.

### Serial Monitor

```bash
pio device monitor --port /dev/ttyACM0 --baud 115200
```

Expected boot output:
```
══════════════════════════════════════
 ESP32-S3 NMEA 2000 GPS Node  v1.0.0
 XH-S3E + HGLRC M100 Pro + SN65HVD230
══════════════════════════════════════
[WiFi] Connected: 192.168.2.124
[OTA] Ready — gps-n2k:3232
[N2K] Bus open — address claiming started.
[GPS] HGLRC M100 Pro (u-blox M10) ready — 10 Hz, SEA model
[BOOT] GPS ready. Waiting for fix...
```

## Web Dashboard

Once connected to WiFi, visit `http://<node-ip>/` (or `http://gps-n2k.local/`) for a live status page:

- Auto-refreshing Leaflet/OpenStreetMap view, centered on first fix
- Fix status, satellite count, HDOP, lat/lon, altitude, SOG, COG
- N2K bus and MQTT connection status
- Uptime and free heap (for spotting memory leaks during long-term testing)

Raw JSON is available at `http://<node-ip>/status.json` for scripting or external monitoring.

> The dashboard works even before a GPS fix is obtained — useful for confirming WiFi/network connectivity while debugging hardware. Map tiles require internet access; the data grid works fully offline on the local network.

## MQTT

Publishes a single JSON payload to `gps-n2k/position` on the Mosquitto broker (`192.168.2.120:1883`) once per second:

```json
{"lat":55.1656591,"lon":11.6686768,"alt_m":50.261,"sog_kn":0.0836,"cog_deg":0,"hdop":1.61,"numSV":15,"fix":true,"time":"2026-06-20T09:17:11Z"}
```

> COG reads `0` at near-zero SOG — this is expected GNSS behaviour (course is undefined/unstable below a minimum speed threshold), not a bug.

Subscribe for testing:
```bash
mosquitto_sub -h 192.168.2.120 -t gps-n2k/position -v
```

## Verifying on the N2K Bus

On a Raspberry Pi with a PiCAN-M or similar:

```bash
candump can0 | grep -E "1F805|1F806|1F80D|1F011"
```

PGN 129025 = `0x1F805`, 129026 = `0x1F806`, 129029 = `0x1F80D`, 126992 = `0x1F011`

## Raymarine Axiom Compatibility

Confirmed working — Axiom (and any standards-compliant N2K/SeaTalkNG MFD) accepts this node as a generic GPS source. SeaTalkNG is electrically and protocol-identical to NMEA 2000; only the connector differs. The node transmits standard (non-proprietary) PGNs, so Axiom treats it the same as a commercial sensor (e.g. Raystar 130).

If another GPS source is also on the bus (e.g. a Vesper AIS transponder), check Axiom's **Data Sources** menu and manually select the preferred source if auto-selection picks the wrong one or flickers between sources.

## Known Issues / Debugging Notes

- **`DaysSince1970` is not a library function** — despite appearing as a parameter name throughout `N2kMessages.h`, there is no built-in date conversion utility in the NMEA2000 library. A local `dateToDaysSince1970()` helper (Howard Hinnant's algorithm) is implemented in `main.cpp`.
- **`gps.begin(Serial1)` can fail to detect the module even with correct wiring/power/baud** if the module is already streaming UBX data continuously at high rate — the library's default handshake polls for a specific ACK that can get lost in the stream. Fix: call `gps.begin(Serial1, 1100, true)` — the `assumeSuccess` flag accepts any valid UBX/NMEA traffic as confirmation rather than requiring a specific poll response.
- **`gps.setAutoPVTcallbackPtr()` must be called *after* `gps.begin()` succeeds**, not before — calling it earlier causes a null-pointer crash (`Guru Meditation Error: StoreProhibited`) since the library's internal callback structures aren't allocated until `.begin()` completes.
- **PlatformIO registry names differ from GitHub repo names**: use `ttlappalainen/NMEA2000-library` (not `NMEA2000`), and `sergei/NMEA2000_esp32_twai` (not under the `ttlappalainen` org).
- **`platformio.ini` comments use `;` not `#`** — `#`-prefixed lines are parsed as invalid config keys, not ignored.

## Notes

- The node will not transmit any PGNs onto the N2K bus until a valid GPS fix is obtained (≥4 satellites, 3D fix) — see `MIN_SATELLITES_FOR_VALID` in `Config.h`
- `N2K_UNIQUE_NUMBER` must be different from all other DIY nodes on the bus. The relay board (`esp32-s3-eth-relay-board`) uses 2001; this node uses 2002
- GPS antenna lateral offset from the vessel's reference point (centerline) does not need correction in this firmware — apply it in Signal K or the MFD's antenna offset setting if needed. Physical orientation of the GPS module/antenna has no effect on COG or SOG, since these are computed from the Doppler/position-fix solution, not physical heading
- WiFi is fully optional — the node functions completely on the N2K bus without any WiFi connection; OTA, MQTT, and the web dashboard are simply unavailable until WiFi is configured