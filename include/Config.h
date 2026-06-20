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
#define TWAI_TX_PIN     GPIO_NUM_4
#define TWAI_RX_PIN     GPIO_NUM_5

// ── GPS UART — HGLRC M100 Pro (u-blox M10) ──────────────────────────────────
#define GPS_UART_PORT       1
#define GPS_UART_RX_PIN     16
#define GPS_UART_TX_PIN     17
#define GPS_BAUD_INITIAL    115200      // M100 Pro factory default (confirmed via UART test)
#define GPS_BAUD_TARGET     115200      // Already at target — no reconfiguration needed
#define GPS_UPDATE_HZ       10

// ── WiFi / OTA ───────────────────────────────────────────────────────────────
// Credentials are NOT stored here — WiFiManager handles them via captive portal.
// On first boot (or after reset), the node broadcasts an AP named WIFI_AP_NAME.
// Connect your phone/laptop to that AP and enter your WiFi credentials.
// They are saved to NVS flash and used automatically on all subsequent boots.
// To re-configure: call wm.resetSettings() or hold a button (future feature).
#define WIFI_AP_NAME            "GPS-N2K-Setup"     // AP name shown on your phone
#define WIFI_PORTAL_TIMEOUT     60                  // Seconds before giving up and continuing without WiFi
#define OTA_HOSTNAME            "gps-n2k"
#define OTA_PORT                3232

// ── MQTT ─────────────────────────────────────────────────────────────────────
// Publishes a single JSON payload per update to the Mosquitto broker on NUC2
#define MQTT_BROKER          "192.168.2.120"
#define MQTT_PORT             1883
#define MQTT_TOPIC_GPS        "gps-n2k/position"
#define INTERVAL_MQTT_MS       1000     // 1 Hz — matches GNSS/SysTime PGN rate

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
