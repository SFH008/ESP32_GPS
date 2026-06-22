#pragma once

// ═══════════════════════════════════════════════════════════════════════════
//  Config.h — All pins, constants, and N2K identity in one place
//
//  Hardware: Sparkle IoT XH-S3E N16R8 (ESP32-S3-WROOM-1, 16MB flash, 8MB PSRAM)
//            SN65HVD230 CAN transceiver
//            HGLRC M100 Pro GPS (u-blox M10, JST-GH 1.25mm connector)
//            STNG drop cable → SN65HVD230 → ESP32-S3 TWAI
//            Active 12V piezo buzzer via IRLZ44N MOSFET on GPIO 8
//            Silence button (momentary, → GND) on GPIO 9
// ═══════════════════════════════════════════════════════════════════════════

// ── TWAI (CAN) — SN65HVD230 transceiver ─────────────────────────────────────
#define TWAI_TX_PIN     GPIO_NUM_4
#define TWAI_RX_PIN     GPIO_NUM_5

// ── GPS UART — HGLRC M100 Pro (u-blox M10) ──────────────────────────────────
#define GPS_UART_PORT       1
#define GPS_UART_RX_PIN     16
#define GPS_UART_TX_PIN     17
#define GPS_BAUD_INITIAL    115200
#define GPS_BAUD_TARGET     115200
#define GPS_UPDATE_HZ       10

// ── Alert Siren — Active 12V piezo via IRLZ44N MOSFET ───────────────────────
#define SIREN_PIN               GPIO_NUM_8   // MOSFET gate (100Ω series resistor)
#define SIREN_BUTTON_PIN        GPIO_NUM_9   // Momentary button → GND, internal pull-up
#define SIREN_TIMEOUT_MS        120000       // Safety auto-off after 120s
#define SIREN_BUTTON_DEBOUNCE   50           // ms

// ── WiFi / OTA ───────────────────────────────────────────────────────────────
#define WIFI_AP_NAME            "GPS-N2K-Setup"
#define WIFI_PORTAL_TIMEOUT     60
#define OTA_HOSTNAME            "gps-n2k"
#define OTA_PORT                3232

// ── MQTT ─────────────────────────────────────────────────────────────────────
#define MQTT_BROKER             "192.168.2.120"
#define MQTT_PORT               1883
#define MQTT_TOPIC_GPS          "gps-n2k/position"
#define MQTT_TOPIC_ALERT_STATE  "gps-n2k/alert/active"    // publish: alert status 1 Hz
#define MQTT_TOPIC_ALERT_SIL    "gps-n2k/alert/silence"   // subscribe: any payload → silence
#define MQTT_TOPIC_ALERT_ACK    "gps-n2k/alert/ack"       // subscribe: any payload → ACK
#define INTERVAL_MQTT_MS        1000

// ── NMEA 2000 Device Identity ─────────────────────────────────────────────────
#define N2K_UNIQUE_NUMBER           2002
#define N2K_MANUFACTURER_CODE       2046
#define N2K_DEVICE_FUNCTION         145
#define N2K_DEVICE_CLASS            60
#define N2K_INDUSTRY_GROUP          4
#define N2K_DEVICE_INSTANCE         0
#define N2K_SYSTEM_INSTANCE         0
#define N2K_DEVICE_MODEL_ID         "XH-S3E-GPS-NODE"
#define N2K_DEVICE_SW_VERSION       "1.1.0"
#define N2K_DEVICE_MODEL_VERSION    "XH-S3E+M100Pro"
#define N2K_DEVICE_SERIAL_CODE      "00000002"
#define N2K_ADDRESS                 35

// ── PGN Transmit Intervals ────────────────────────────────────────────────────
#define INTERVAL_PGN_129025     100
#define INTERVAL_PGN_129026     100
#define INTERVAL_PGN_129029    1000
#define INTERVAL_PGN_126992    1000

// ── GPS Fix Quality ───────────────────────────────────────────────────────────
#define MIN_SATELLITES_FOR_VALID    4