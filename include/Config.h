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
#define TWAI_TX_PIN     GPIO_NUM_4
#define TWAI_RX_PIN     GPIO_NUM_5

// ── GPS UART — HGLRC M100 Pro (u-blox M10) ──────────────────────────────────
// JST-GH 1.25mm 6-pin: 5V, GND, TX, RX (+ compass SDA/SCL, unused)
// M100 Pro power → 5V (VBUS on DevKitC-1), NOT 3.3V
// UART logic is 3.3V — connect directly, no level shifter needed
// GPIO16 ← TX pin on M100 Pro
// GPIO17 → RX pin on M100 Pro
#define GPS_UART_PORT       1           // Serial1
#define GPS_UART_RX_PIN     16
#define GPS_UART_TX_PIN     17
#define GPS_BAUD_INITIAL    38400       // M10 factory default
#define GPS_BAUD_TARGET     115200      // Reconfigured on first boot
#define GPS_UPDATE_HZ       10          // 10 Hz

// ── NMEA 2000 Device Identity ─────────────────────────────────────────────────
#define N2K_UNIQUE_NUMBER           2002
#define N2K_MANUFACTURER_CODE       2046
#define N2K_DEVICE_FUNCTION         145     // GPS
#define N2K_DEVICE_CLASS            60      // Navigation
#define N2K_INDUSTRY_GROUP          4       // Marine
#define N2K_DEVICE_INSTANCE         0
#define N2K_SYSTEM_INSTANCE         0
#define N2K_DEVICE_MODEL_ID         "XH-S3E-GPS-NODE"
#define N2K_DEVICE_SW_VERSION       "1.0.0"
#define N2K_DEVICE_MODEL_VERSION    "XH-S3E+M100Pro"
#define N2K_DEVICE_SERIAL_CODE      "00000002"
#define N2K_ADDRESS                 35

// ── PGN Transmit Intervals ────────────────────────────────────────────────────
#define INTERVAL_PGN_129025     100     // 10 Hz
#define INTERVAL_PGN_129026     100     // 10 Hz
#define INTERVAL_PGN_129029    1000     //  1 Hz
#define INTERVAL_PGN_126992    1000     //  1 Hz

// ── GPS Fix Quality ───────────────────────────────────────────────────────────
#define MIN_SATELLITES_FOR_VALID    4

// ── OTA / WiFi ────────────────────────────────────────────────────────────────
#define WIFI_SSID           "your-ssid"
#define WIFI_PASSWORD       "your-password"
#define OTA_HOSTNAME        "gps-n2k"
#define OTA_PORT            3232
