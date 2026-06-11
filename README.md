# ESP32-S3 NMEA 2000 GPS Node

A lightweight, standalone NMEA 2000 GPS node built on the ESP32-S3. Reads position data from a u-blox M10 GPS module and injects it onto a Raymarine SeaTalkNG / NMEA 2000 bus at 10 Hz. Designed for boats where the chart plotter's internal GPS is unreliable or absent.

## Features

- Transmits four PGNs onto the N2K bus:
  - **129025** — Position, Rapid Update (10 Hz)
  - **129026** — COG & SOG, Rapid Update (10 Hz)
  - **129029** — GNSS Position Data (1 Hz)
  - **126992** — System Time (1 Hz)
- u-blox M10 configured for UBX binary, 10 Hz updates, maritime dynamic model
- Full NMEA 2000 address claiming
- OTA firmware updates over WiFi
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

The SeaTalkNG connector carries standard NMEA 2000 signals:

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

The M100 Pro uses a JST-GH 1.25mm 6-pin connector. Only 4 wires are used (compass SDA/SCL are not connected):

| M100 Pro | ESP32-S3 |
|---|---|
| 5V | 5V (VBUS) |
| GND | GND |
| TX | GPIO16 |
| RX | GPIO17 |

> The M10 UART is 3.3V logic — connect directly to the ESP32-S3, no level shifter needed. Power must be 5V, not 3.3V.

## Pin Summary

| Function | GPIO |
|---|---|
| TWAI TX (→ SN65HVD230 TXD) | GPIO4 |
| TWAI RX (← SN65HVD230 RXD) | GPIO5 |
| GPS UART RX (← M100 Pro TX) | GPIO16 |
| GPS UART TX (→ M100 Pro RX) | GPIO17 |

## Software

Built with [PlatformIO](https://platformio.org/) using the [pioarduino](https://github.com/pioarduino/platform-espressif32) platform fork.

### Dependencies

- [ttlappalainen/NMEA2000-library](https://registry.platformio.org/libraries/ttlappalainen/NMEA2000-library)
- [sergei/NMEA2000_esp32_twai](https://github.com/sergei/NMEA2000_esp32_twai)
- [sparkfun/SparkFun u-blox GNSS v3](https://registry.platformio.org/libraries/sparkfun/SparkFun%20u-blox%20GNSS%20v3)

### Configuration

Edit `include/Config.h` before building:

```cpp
// WiFi credentials for OTA
#define WIFI_SSID       "your-ssid"
#define WIFI_PASSWORD   "your-password"

// N2K unique number — must be unique across all nodes on your bus
#define N2K_UNIQUE_NUMBER   2002
```

### Building and Flashing

**First flash** — comment out `upload_protocol = espota` in `platformio.ini`, connect via USB and run:

```bash
pio run -t upload
```

**Subsequent updates** — OTA over WiFi once the node is running:

```bash
pio run -t upload   # upload_protocol = espota must be active
```

OTA hostname: `gps-n2k.local` (port 3232)

### Serial Monitor

```bash
pio device monitor
```

Expected boot output:
```
══════════════════════════════════════
 ESP32-S3 NMEA 2000 GPS Node  v1.0.0
 XH-S3E + HGLRC M100 Pro + SN65HVD230
══════════════════════════════════════
[WiFi] Connected: 192.168.x.x
[N2K] Bus open — address claiming started.
[GPS] HGLRC M100 Pro (u-blox M10) ready — 10 Hz, SEA model
[BOOT] GPS ready. Waiting for fix...
```

## Verifying on the Bus

On a Raspberry Pi with a PiCAN-M or similar:

```bash
candump can0 | grep -E "1F805|1F806|1F80D|1F011"
```

PGN 129025 = `0x1F805`, 129026 = `0x1F806`, 129029 = `0x1F80D`, 126992 = `0x1F011`

## Notes

- The node will not transmit any PGNs until a valid GPS fix is obtained (≥4 satellites, 3D fix)
- On first boot the M10 is reconfigured from factory baud (38400) to 115200 and set to UBX binary output only. This config is saved to the module's flash — subsequent boots connect at 115200 directly
- `N2K_UNIQUE_NUMBER` must be different from all other DIY nodes on the bus. The relay board uses 2001; this node uses 2002
- WiFi is optional — the node functions fully on the N2K bus without a WiFi connection; OTA is simply unavailable until WiFi connects

Wire it up — SN65HVD230 to GPIO4/5, M100 Pro JST-GH to GPIO16/17 + 5V, STNG drop cable to transceiver
Fill in WiFi credentials in Config.h
First flash via USB: temporarily comment out upload_protocol = espota in platformio.ini, then pio run -t upload
Check serial monitor for [GPS] and [N2K] boot messages
Verify PGNs on the bus with candump can0 on the Pi


GPIO4, GPIO5 (TWAI) — listed as first-priority free GPIOs with no special functions or restrictions. Safe for TWAI TX/RX. Manuals+
GPIO16, GPIO17 (GPS UART) — also listed as first-priority free GPIOs. UART1 defaults to GPIO17 TX / GPIO18 RX on the ESP32-S3, but can be remapped via the GPIO Matrix — our code assigns Serial1 explicitly to GPIO16/17 which overrides the default, so no conflict. Manuals+Manuals+
Pins to avoid — GPIO26–32 are reserved for SPI flash and PSRAM, and on the XH-S3E N16R8 with Octal PSRAM, GPIO33–37 are also consumed by SPIIO4–SPIIO7 and SPIDQS. GPIO0, 3, 45, 46 are strapping pins to avoid, and GPIO19/20 are native USB. Pimoroni + 2
