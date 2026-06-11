#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <NMEA2000_esp32_twai.h>
#include <N2kMessages.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "Config.h"

// ============================================================
// Globals
// ============================================================
tNMEA2000_esp32_twai NMEA2000(TWAI_TX_PIN, TWAI_RX_PIN);
SFE_UBLOX_GNSS       gps;

// Timing
uint32_t lastPos     = 0;
uint32_t lastCogSog  = 0;
uint32_t lastGnss    = 0;
uint32_t lastSysTime = 0;

// Latest GPS state — updated by uBlox library callbacks
struct GpsData {
    double   lat       = 0.0;
    double   lon       = 0.0;
    double   altMSL    = 0.0;    // metres
    double   sogMs     = 0.0;    // m/s
    double   cogRad    = 0.0;    // radians
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

// ============================================================
// N2K device info
// ============================================================
const unsigned long N2K_MFGCODE   = 2046;   // NMEA reserved "experimental"
const unsigned char N2K_DEVCLASS  = 60;     // Navigation
const unsigned char N2K_DEVFUNC   = 145;    // Ownship position (GNSS)
const unsigned char N2K_INDUSTRY  = 4;      // Marine

// ============================================================
// uBlox callback — fires at GPS_UPDATE_HZ
// ============================================================
void onGpsData(UBX_NAV_PVT_data_t *ubxData) {
    if (ubxData == nullptr) return;

    bool fixOk = (ubxData->flags.bits.gnssFixOK) && (ubxData->fixType >= 3);
    gpsData.valid   = fixOk;
    gpsData.numSV   = ubxData->numSV;
    gpsData.year    = ubxData->year;
    gpsData.month   = ubxData->month;
    gpsData.day     = ubxData->day;
    gpsData.hour    = ubxData->hour;
    gpsData.minute  = ubxData->min;
    gpsData.second  = ubxData->sec;

    if (fixOk) {
        gpsData.lat    = ubxData->lat * 1e-7;                  // degrees
        gpsData.lon    = ubxData->lon * 1e-7;                  // degrees
        gpsData.altMSL = ubxData->hMSL * 1e-3;                // mm → m
        gpsData.sogMs  = ubxData->gSpeed * 1e-3;              // mm/s → m/s
        // headMot in degrees *1e-5, convert to radians
        gpsData.cogRad = (ubxData->headMot * 1e-5) * DEG_TO_RAD;
        gpsData.hdop   = ubxData->pDOP * 0.01;
    }
}

// ============================================================
// PGN senders
// ============================================================
void sendPGN129025() {
    // Rapid position update
    tN2kMsg msg;
    SetN2kLatLonRapid(msg, gpsData.lat, gpsData.lon);
    NMEA2000.SendMsg(msg);
}

void sendPGN129026() {
    // COG & SOG rapid update
    tN2kMsg msg;
    SetN2kCOGSOGRapid(msg, 1,
        N2khr_true,
        gpsData.cogRad,
        gpsData.sogMs);
    NMEA2000.SendMsg(msg);
}

void sendPGN129029() {
    // Full GNSS position data
    tN2kMsg msg;
    SetN2kGNSS(msg,
        1,                              // SID
        DaysSince1970(gpsData.year, gpsData.month, gpsData.day),
        (gpsData.hour * 3600.0) + (gpsData.minute * 60.0) + gpsData.second,
        gpsData.lat,
        gpsData.lon,
        gpsData.altMSL,
        N2kGNSSt_GPS,                  // type
        N2kGNSSm_GNSSfix,              // method
        gpsData.numSV,
        gpsData.hdop,
        0.0,                           // PDOP (not available from PVT directly)
        0.0,                           // geoidal separation
        0,                             // reference stations
        N2kGNSSt_GPS,
        0,
        0.0
    );
    NMEA2000.SendMsg(msg);
}

void sendPGN126992() {
    // System time
    tN2kMsg msg;
    SetN2kSystemTime(msg,
        1,
        DaysSince1970(gpsData.year, gpsData.month, gpsData.day),
        (gpsData.hour * 3600.0) + (gpsData.minute * 60.0) + gpsData.second
    );
    NMEA2000.SendMsg(msg);
}

// ============================================================
// WiFi + OTA
// ============================================================
void setupWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        tries++;
    }
    // Continue even if WiFi fails — N2K works without it
}

void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.begin();
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);

    // --- WiFi + OTA ---
    setupWifi();
    setupOTA();

    // --- NEO-M9N on Serial1 ---
    Serial1.begin(GPS_UART_BAUD, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);

    if (!gps.begin(Serial1)) {
        Serial.println("NEO-M9N not detected — check wiring");
        // Retry loop; don't proceed to N2K without GPS
        while (true) {
            ArduinoOTA.handle();
            delay(1000);
        }
    }

    // Configure NEO-M9N
    gps.setUART1Output(COM_TYPE_UBX);              // UBX binary only, no NMEA noise
    gps.setNavigationFrequency(GPS_UPDATE_HZ);
    gps.setDynamicModel(DYN_MODEL_SEA);            // Maritime dynamic model
    gps.setAutoPVTcallbackPtr(&onGpsData);         // Async callback
    gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Persist baud/output settings

    // --- NMEA 2000 ---
    NMEA2000.SetProductInformation(
        "001",                  // Serial
        1,                      // Product code
        N2K_DEVICE_NAME,
        "1.0.0",                // Firmware version
        "1.0.0"                 // Model version
    );
    NMEA2000.SetDeviceInformation(
        100001,                 // Unique device number
        N2K_DEVFUNC,
        N2K_DEVCLASS,
        N2K_MFGCODE,
        N2K_INDUSTRY
    );
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, N2K_ADDRESS);
    NMEA2000.EnableForward(false);
    NMEA2000.Open();

    Serial.println("GPS N2K node started");
}

// ============================================================
// Loop
// ============================================================
void loop() {
    ArduinoOTA.handle();
    gps.checkUblox();       // Feed the uBlox parser — fires callback when PVT ready
    gps.checkCallbacks();   // Execute any pending callbacks
    NMEA2000.ParseMessages();

    uint32_t now = millis();

    if (!gpsData.valid) return;  // Nothing to transmit without a fix

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