#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ── NMEA 2000 via TWAI ───────────────────────────────────────────────────────
// Do NOT use NMEA2000_CAN.h — on ESP32 it selects the wrong (SPI/MCP2515) driver.
// Instantiate TWAI driver directly with pins from Config.h.
#include <NMEA2000_esp32_twai.h>
#include <N2kMessages.h>

// ── u-blox GNSS ──────────────────────────────────────────────────────────────
#include <SparkFun_u-blox_GNSS_v3.h>

#include "Config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  Globals
// ═══════════════════════════════════════════════════════════════════════════

tNMEA2000_esp32_twai NMEA2000(TWAI_TX_PIN, TWAI_RX_PIN);
SFE_UBLOX_GNSS_SERIAL gps;

// Transmit timing
uint32_t lastPos     = 0;
uint32_t lastCogSog  = 0;
uint32_t lastGnss    = 0;
uint32_t lastSysTime = 0;

// GPS state — populated by UBX NAV-PVT callback
struct GpsData {
    double   lat       = 0.0;
    double   lon       = 0.0;
    double   altMSL    = 0.0;   // metres above MSL
    double   sogMs     = 0.0;   // m/s
    double   cogRad    = 0.0;   // radians true
    double   hdop      = 9.9;
    uint8_t  numSV     = 0;
    uint16_t year      = 0;
    uint8_t  month     = 0;
    uint8_t  day       = 0;
    uint8_t  hour      = 0;
    uint8_t  minute    = 0;
    uint8_t  second    = 0;
    bool     valid     = false;
} gpsData;

// PGNs this node transmits — required for N2K address claiming
const unsigned long TX_PGN_LIST[] PROGMEM = {
    126992L,    // System Time
    129025L,    // Position, Rapid Update
    129026L,    // COG & SOG, Rapid Update
    129029L,    // GNSS Position Data
    0
};

// ═══════════════════════════════════════════════════════════════════════════
//  UBX NAV-PVT callback — fires at GPS_UPDATE_HZ
// ═══════════════════════════════════════════════════════════════════════════

void onGpsData(UBX_NAV_PVT_data_t *d) {
    if (d == nullptr) return;

    bool fixOk      = d->flags.bits.gnssFixOK && (d->fixType >= 3)
                      && (d->numSV >= MIN_SATELLITES_FOR_VALID);
    gpsData.valid   = fixOk;
    gpsData.numSV   = d->numSV;
    gpsData.year    = d->year;
    gpsData.month   = d->month;
    gpsData.day     = d->day;
    gpsData.hour    = d->hour;
    gpsData.minute  = d->min;
    gpsData.second  = d->sec;

    if (fixOk) {
        gpsData.lat    = d->lat    * 1e-7;          // 1e-7 deg → degrees
        gpsData.lon    = d->lon    * 1e-7;
        gpsData.altMSL = d->hMSL  * 1e-3;          // mm → m
        gpsData.sogMs  = d->gSpeed * 1e-3;          // mm/s → m/s
        gpsData.cogRad = (d->headMot * 1e-5) * DEG_TO_RAD; // 1e-5 deg → rad
        gpsData.hdop   = d->pDOP  * 0.01;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  PGN senders
// ═══════════════════════════════════════════════════════════════════════════

void sendPGN129025() {
    tN2kMsg msg;
    SetN2kLatLonRapid(msg, gpsData.lat, gpsData.lon);
    NMEA2000.SendMsg(msg);
}

void sendPGN129026() {
    tN2kMsg msg;
    SetN2kCOGSOGRapid(msg, 1, N2khr_true, gpsData.cogRad, gpsData.sogMs);
    NMEA2000.SendMsg(msg);
}

void sendPGN129029() {
    tN2kMsg msg;
    SetN2kGNSS(msg,
        1,                          // SID
        DaysSince1970(gpsData.year, gpsData.month, gpsData.day),
        (gpsData.hour * 3600.0) + (gpsData.minute * 60.0) + gpsData.second,
        gpsData.lat,
        gpsData.lon,
        gpsData.altMSL,
        N2kGNSSt_GPS,               // GNSS type
        N2kGNSSm_GNSSfix,           // method
        gpsData.numSV,
        gpsData.hdop,
        0.0,                        // PDOP — not in NAV-PVT, omit
        0.0,                        // geoidal separation
        0,                          // reference stations
        N2kGNSSt_GPS,
        0,
        0.0
    );
    NMEA2000.SendMsg(msg);
}

void sendPGN126992() {
    tN2kMsg msg;
    SetN2kSystemTime(msg,
        1,
        DaysSince1970(gpsData.year, gpsData.month, gpsData.day),
        (gpsData.hour * 3600.0) + (gpsData.minute * 60.0) + gpsData.second
    );
    NMEA2000.SendMsg(msg);
}

// ═══════════════════════════════════════════════════════════════════════════
//  WiFi + OTA
// ═══════════════════════════════════════════════════════════════════════════

void setupWiFiAndOTA() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);

    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Failed — OTA unavailable. Continuing without.");
        return;
    }
    Serial.printf("\n[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.onStart([]()   { Serial.println("[OTA] Starting..."); });
    ArduinoOTA.onEnd([]()     { Serial.println("\n[OTA] Done. Rebooting."); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        Serial.printf("[OTA] %u%%\r", p * 100 / t);
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("[OTA] Error[%u]\n", e);
    });
    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready — %s:%d\n", OTA_HOSTNAME, OTA_PORT);
}

// ═══════════════════════════════════════════════════════════════════════════
//  NMEA 2000
// ═══════════════════════════════════════════════════════════════════════════

void setupNMEA2000() {
    NMEA2000.SetProductInformation(
        N2K_DEVICE_SERIAL_CODE,
        100,
        N2K_DEVICE_MODEL_ID,
        N2K_DEVICE_SW_VERSION,
        N2K_DEVICE_MODEL_VERSION
    );
    NMEA2000.SetDeviceInformation(
        N2K_UNIQUE_NUMBER,
        N2K_DEVICE_FUNCTION,        // 145 = GPS
        N2K_DEVICE_CLASS,           // 60  = Navigation
        N2K_MANUFACTURER_CODE
    );
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, N2K_ADDRESS);
    NMEA2000.ExtendTransmitMessages(TX_PGN_LIST);
    NMEA2000.EnableForward(false);
    NMEA2000.Open();
    Serial.println("[N2K] Bus open — address claiming started.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  GPS init — baud negotiation + M10 configuration
// ═══════════════════════════════════════════════════════════════════════════

void setupGPS() {
    // Start at factory default baud, then switch to target
    Serial1.begin(GPS_BAUD_INITIAL, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);

    if (!gps.begin(Serial1)) {
        // Try target baud in case it was already configured (warm boot / saved config)
        Serial1.end();
        Serial1.begin(GPS_BAUD_TARGET, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
        if (!gps.begin(Serial1)) {
            Serial.println("[GPS] *** Not detected — check JST-GH wiring and 5V supply ***");
            while (true) {
                ArduinoOTA.handle();
                delay(1000);
            }
        }
    }

    // Switch to target baud for higher throughput
    gps.setSerialRate(GPS_BAUD_TARGET);
    Serial1.end();
    Serial1.begin(GPS_BAUD_TARGET, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
    gps.begin(Serial1);

    gps.setUART1Output(COM_TYPE_UBX);               // UBX binary only — suppress NMEA
    gps.setNavigationFrequency(GPS_UPDATE_HZ);      // 10 Hz
    gps.setDynamicModel(DYN_MODEL_SEA);             // Maritime motion model
    gps.setAutoPVTcallbackPtr(&onGpsData);          // Async NAV-PVT callback
    gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Persist UART config across reboot

    Serial.printf("[GPS] HGLRC M100 Pro (u-blox M10) ready — %d Hz, SEA model\n",
                  GPS_UPDATE_HZ);
}

// ═══════════════════════════════════════════════════════════════════════════
//  setup()
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n══════════════════════════════════════");
    Serial.println(" ESP32-S3 NMEA 2000 GPS Node  v1.0.0");
    Serial.println(" XH-S3E + HGLRC M100 Pro + SN65HVD230");
    Serial.println("══════════════════════════════════════");

    setupWiFiAndOTA();
    setupNMEA2000();
    setupGPS();

    Serial.println("[BOOT] GPS ready. Waiting for fix...");
}

// ═══════════════════════════════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    ArduinoOTA.handle();

    gps.checkUblox();       // Feed UBX parser — fires onGpsData() when NAV-PVT ready
    gps.checkCallbacks();   // Execute pending callbacks on this core

    NMEA2000.ParseMessages(); // Service N2K bus — essential for address claiming

    if (!gpsData.valid) return;

    uint32_t now = millis();

    if (now - lastPos >= INTERVAL_PGN_129025) {
        lastPos = now;
        sendPGN129025();
    }
    if (now - lastCogSog >= INTERVAL_PGN_129026) {
        lastCogSog = now;
        sendPGN129026();
    }
    if (now - lastGnss >= INTERVAL_PGN_129029) {
        lastGnss = now;
        sendPGN129029();
    }
    if (now - lastSysTime >= INTERVAL_PGN_126992) {
        lastSysTime = now;
        sendPGN126992();
    }
}