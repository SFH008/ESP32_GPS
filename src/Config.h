#pragma once

// ═══════════════════════════════════════════════════════════════════════════
//  Config.h — All pins, constants, and N2K identity in one place
//
//  Hardware: Sparkle IoT XH-S3E N16R8 (ESP32-S3-WROOM-1, 16MB flash, 8MB PSRAM)
//            SN65HVD230 CAN transceiver
//            HGLRC M100 Pro GPS (u-blox M10, JST-GH 1.25mm connector)
//            STNG drop cable → SN65HVD230 → ESP32-S3 TWAI
// ═══════════════════════════════════════════════════════════════════════════

// ── TWAI (CAN) — SN65HVD230 transceiver ─────────────────────────────────────
// GPIO4 → TXD on SN65HVD230
// GPIO5 ← RXD on SN65HVD230
// Rs pin on SN65HVD230 → GND (normal speed, not slope-controlled)
// Passed as constructor args to tNMEA2000_esp32_twai in main.cpp
#define TWAI_TX_PIN     GPIO_NUM_4
#define TWAI_RX_PIN     GPIO_NUM_5

// ── GPS UART — HGLRC M100 Pro (u-blox M10) ──────────────────────────────────
// JST-GH 1.25mm 6-pin cable: 5V, GND, TX, RX (+ compass SDA/SCL, unused)
// M100 Pro power → 5V (VBUS on DevKitC-1), NOT 3.3V
// UART logic is 3.3V — connect directly, no level shifter needed
// GPIO16 ← TX pin on M100 Pro (M10 transmits position data)
// GPIO17 → RX pin on M100 Pro (ESP32 sends UBX config commands)
#define GPS_UART_PORT       1           // Serial1
#define GPS_UART_RX_PIN     16
#define GPS_UART_TX_PIN     17
#define GPS_BAUD_INITIAL    38400       // M10 factory default baud
#define GPS_BAUD_TARGET     115200      // Reconfigured on first boot via UBX
#define GPS_UPDATE_HZ       10          // 10 Hz — M10 supports up to 25 Hz

// ── NMEA 2000 Device Identity ─────────────────────────────────────────────────
// N2K_UNIQUE_NUMBER must be unique across all N2K nodes on the bus (0–2097151)
#define N2K_UNIQUE_NUMBER           2002        // Different from relay board (2001 if used)
#define N2K_MANUFACTURER_CODE       2046        // 0x7FE = DIY/unknown
#define N2K_DEVICE_FUNCTION         145         // GPS — N2K device function table
#define N2K_DEVICE_CLASS            60          // Navigation
#define N2K_INDUSTRY_GROUP          4           // Marine
#define N2K_DEVICE_INSTANCE         0
#define N2K_SYSTEM_INSTANCE         0
#define N2K_DEVICE_MODEL_ID         "XH-S3E-GPS-NODE"
#define N2K_DEVICE_SW_VERSION       "1.0.0"
#define N2K_DEVICE_MODEL_VERSION    "XH-S3E+M100Pro"
#define N2K_DEVICE_SERIAL_CODE      "00000002"
#define N2K_ADDRESS                 35          // Preferred address — negotiates if taken

// ── PGN Transmit Intervals ────────────────────────────────────────────────────
#define INTERVAL_PGN_129025     100     // 10 Hz — Rapid position update
#define INTERVAL_PGN_129026     100     // 10 Hz — COG & SOG rapid
#define INTERVAL_PGN_129029    1000     //  1 Hz — Full GNSS position data
#define INTERVAL_PGN_126992    1000     //  1 Hz — System time

// ── GPS Fix Quality ───────────────────────────────────────────────────────────
#define MIN_SATELLITES_FOR_VALID    4

// ── OTA / WiFi ────────────────────────────────────────────────────────────────
#define WIFI_SSID           "Network_2.4"
#define WIFI_PASSWORD       "dots1234"
#define OTA_HOSTNAME        "gps-n2k"
#define OTA_PORT            3232