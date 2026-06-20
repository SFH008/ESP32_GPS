#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── NMEA 2000 via TWAI ───────────────────────────────────────────────────────
#include <NMEA2000_esp32_twai.h>
#include <N2kMessages.h>
#include <N2kDef.h>

// ── u-blox GNSS ──────────────────────────────────────────────────────────────
#include <SparkFun_u-blox_GNSS_v3.h>

#include "Config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  Globals
// ═══════════════════════════════════════════════════════════════════════════

NMEA2000_esp32_twai  NMEA2000(TWAI_TX_PIN, TWAI_RX_PIN);
SFE_UBLOX_GNSS_SERIAL gps;

WiFiClient   espClient;
PubSubClient mqttClient(espClient);
WebServer    webServer(80);

bool wifiConnected = false;

uint32_t lastPos     = 0;
uint32_t lastCogSog  = 0;
uint32_t lastGnss    = 0;
uint32_t lastSysTime = 0;
uint32_t lastMqtt    = 0;

struct GpsData {
    double   lat       = 0.0;
    double   lon       = 0.0;
    double   altMSL    = 0.0;
    double   geoidSep  = 0.0;
    double   sogMs     = 0.0;
    double   cogRad    = 0.0;
    double   hdop      = 9.9;
    double   pdop      = 9.9;
    uint8_t  numSV     = 0;
    uint16_t year      = 0;
    uint8_t  month     = 0;
    uint8_t  day       = 0;
    uint8_t  hour      = 0;
    uint8_t  minute    = 0;
    uint8_t  second    = 0;
    bool     valid     = false;
} gpsData;

const unsigned long TX_PGN_LIST[] PROGMEM = {
    126992L, 129025L, 129026L, 129029L, 0
};

// ═══════════════════════════════════════════════════════════════════════════
//  Date helper
// ═══════════════════════════════════════════════════════════════════════════

static uint16_t dateToDaysSince1970(uint16_t year, uint8_t month, uint8_t day) {
    uint32_t y = year, m = month, d = day;
    if (m <= 2) { y--; m += 12; }
    uint32_t era = y / 400;
    uint32_t yoe = y - era * 400;
    uint32_t doy = (153 * (m - 3) + 2) / 5 + d - 1;
    uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (uint16_t)(era * 146097 + doe - 719468);
}

// ═══════════════════════════════════════════════════════════════════════════
//  UBX NAV-PVT callback
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
        gpsData.lat      = d->lat    * 1e-7;
        gpsData.lon      = d->lon    * 1e-7;
        gpsData.altMSL   = d->hMSL  * 1e-3;
        // Geoidal separation = height above ellipsoid - height above MSL.
        // Both fields are present in every NAV-PVT message, no extra UBX
        // query needed. This is what was previously hardcoded to 0.0 in
        // PGN 129029, and explains the inflated altitude readings during
        // initial bring-up — Axiom/Signal K need this to correctly relate
        // ellipsoidal height (what the GPS measures) to MSL (what charts use).
        gpsData.geoidSep = (d->height - d->hMSL) * 1e-3;
        gpsData.sogMs    = d->gSpeed * 1e-3;
        gpsData.cogRad   = (d->headMot * 1e-5) * DEG_TO_RAD;
        // pDOP from NAV-PVT used as a fallback only — NAV-DOP below is authoritative
        // and provides true HDOP separately (NAV-PVT only carries PDOP)
        gpsData.pdop     = d->pDOP  * 0.01;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  UBX NAV-DOP callback — authoritative HDOP/PDOP source
//  Required for Axiom's "meters of accuracy" estimate (PGN 129029 fields 12+13)
// ═══════════════════════════════════════════════════════════════════════════

void onGpsDop(UBX_NAV_DOP_data_t *d) {
    if (d == nullptr) return;
    gpsData.hdop = d->hDOP * 0.01;   // scale factor 0.01 per UBX spec
    gpsData.pdop = d->pDOP * 0.01;
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
    SetN2kGNSS(msg, 1,
        dateToDaysSince1970(gpsData.year, gpsData.month, gpsData.day),
        (gpsData.hour * 3600.0) + (gpsData.minute * 60.0) + gpsData.second,
        gpsData.lat, gpsData.lon, gpsData.altMSL,
        N2kGNSSt_GPS, N2kGNSSm_GNSSfix,
        gpsData.numSV, gpsData.hdop,
        gpsData.pdop, gpsData.geoidSep, 0, N2kGNSSt_GPS, 0, 0.0
    );
    NMEA2000.SendMsg(msg);
}

void sendPGN126992() {
    tN2kMsg msg;
    SetN2kSystemTime(msg, 1,
        dateToDaysSince1970(gpsData.year, gpsData.month, gpsData.day),
        (gpsData.hour * 3600.0) + (gpsData.minute * 60.0) + gpsData.second
    );
    NMEA2000.SendMsg(msg);
}

// ═══════════════════════════════════════════════════════════════════════════
//  MQTT
// ═══════════════════════════════════════════════════════════════════════════

void setupMqtt() {
    if (!wifiConnected) return;
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

void mqttReconnect() {
    if (mqttClient.connected()) return;
    if (mqttClient.connect(OTA_HOSTNAME)) {
        Serial.println("[MQTT] Connected.");
    }
    // Non-blocking — if it fails, we just try again on the next loop interval
}

void publishMqtt() {
    if (!wifiConnected) return;
    if (!mqttClient.connected()) {
        mqttReconnect();
        if (!mqttClient.connected()) return;
    }

    JsonDocument doc;
    doc["lat"]     = gpsData.lat;
    doc["lon"]     = gpsData.lon;
    doc["alt_m"]   = gpsData.altMSL;
    doc["geoid_sep_m"] = gpsData.geoidSep;
    doc["sog_kn"]  = gpsData.sogMs * 1.94384;
    doc["cog_deg"] = gpsData.cogRad * RAD_TO_DEG;
    doc["hdop"]    = gpsData.hdop;
    doc["pdop"]    = gpsData.pdop;
    doc["numSV"]   = gpsData.numSV;
    doc["fix"]     = gpsData.valid;
    doc["time"]    = String(gpsData.year) + "-" +
                      (gpsData.month < 10 ? "0" : "") + String(gpsData.month) + "-" +
                      (gpsData.day   < 10 ? "0" : "") + String(gpsData.day)   + "T" +
                      (gpsData.hour   < 10 ? "0" : "") + String(gpsData.hour)   + ":" +
                      (gpsData.minute < 10 ? "0" : "") + String(gpsData.minute) + ":" +
                      (gpsData.second < 10 ? "0" : "") + String(gpsData.second) + "Z";

    char payload[256];
    size_t n = serializeJson(doc, payload);
    mqttClient.publish(MQTT_TOPIC_GPS, payload, n);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Web Dashboard — status page with auto-refreshing map
// ═══════════════════════════════════════════════════════════════════════════

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GPS N2K Node</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  body { font-family: -apple-system, sans-serif; margin: 0; background: #0d1b2a; color: #e0e6ed; }
  header { padding: 14px 18px; background: #13283f; border-bottom: 1px solid #1f3a5a; }
  h1 { font-size: 18px; margin: 0; }
  #map { height: 320px; width: 100%; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 1px; background: #1f3a5a; }
  .cell { background: #0d1b2a; padding: 12px 14px; }
  .label { font-size: 11px; color: #7a93ab; text-transform: uppercase; letter-spacing: 0.5px; }
  .value { font-size: 20px; font-weight: 600; margin-top: 4px; }
  .ok { color: #4ade80; }
  .bad { color: #f87171; }
  footer { padding: 10px 18px; font-size: 11px; color: #5a7290; }
</style>
</head>
<body>
<header><h1>⚓ GPS N2K Node — XH-S3E + M100 Pro</h1></header>
<div id="map"></div>
<div class="grid" id="grid"></div>
<footer id="footer"></footer>
<script>
let map = L.map('map').setView([0,0], 2);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);
let marker = null;
let firstFix = true;

async function refresh() {
  try {
    const r = await fetch('/status.json');
    const d = await r.json();

    if (d.fix) {
      if (!marker) {
        marker = L.marker([d.lat, d.lon]).addTo(map);
      } else {
        marker.setLatLng([d.lat, d.lon]);
      }
      if (firstFix) { map.setView([d.lat, d.lon], 15); firstFix = false; }
    }

    document.getElementById('grid').innerHTML = `
      <div class="cell"><div class="label">Fix</div><div class="value ${d.fix ? 'ok' : 'bad'}">${d.fix ? 'YES' : 'NO FIX'}</div></div>
      <div class="cell"><div class="label">Satellites</div><div class="value">${d.numSV}</div></div>
      <div class="cell"><div class="label">HDOP</div><div class="value">${d.hdop.toFixed(2)}</div></div>
      <div class="cell"><div class="label">PDOP</div><div class="value">${d.pdop.toFixed(2)}</div></div>
      <div class="cell"><div class="label">Latitude</div><div class="value">${d.lat.toFixed(6)}</div></div>
      <div class="cell"><div class="label">Longitude</div><div class="value">${d.lon.toFixed(6)}</div></div>
      <div class="cell"><div class="label">Altitude</div><div class="value">${d.alt_m.toFixed(1)} m</div></div>
      <div class="cell"><div class="label">SOG</div><div class="value">${d.sog_kn.toFixed(2)} kn</div></div>
      <div class="cell"><div class="label">COG</div><div class="value">${d.cog_deg.toFixed(1)}°</div></div>
      <div class="cell"><div class="label">N2K Bus</div><div class="value ok">OPEN</div></div>
      <div class="cell"><div class="label">MQTT</div><div class="value ${d.mqtt ? 'ok' : 'bad'}">${d.mqtt ? 'CONNECTED' : 'DOWN'}</div></div>
      <div class="cell"><div class="label">Uptime</div><div class="value">${d.uptime_s}s</div></div>
      <div class="cell"><div class="label">GPS Time</div><div class="value">${d.time}</div></div>
    `;
    document.getElementById('footer').textContent =
      `IP ${d.ip} · OTA :${d.ota_port} · Free heap ${d.heap} bytes · Refreshed ${new Date().toLocaleTimeString()}`;
  } catch (e) {
    document.getElementById('footer').textContent = 'Connection lost — retrying...';
  }
}
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
    webServer.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleStatusJson() {
    JsonDocument doc;
    doc["fix"]     = gpsData.valid;
    doc["lat"]     = gpsData.lat;
    doc["lon"]     = gpsData.lon;
    doc["alt_m"]   = gpsData.altMSL;
    doc["geoid_sep_m"] = gpsData.geoidSep;
    doc["sog_kn"]  = gpsData.sogMs * 1.94384;
    doc["cog_deg"] = gpsData.cogRad * RAD_TO_DEG;
    doc["hdop"]    = gpsData.hdop;
    doc["pdop"]    = gpsData.pdop;
    doc["numSV"]   = gpsData.numSV;
    doc["mqtt"]    = mqttClient.connected();
    doc["uptime_s"] = millis() / 1000;
    doc["ip"]       = WiFi.localIP().toString();
    doc["ota_port"] = OTA_PORT;
    doc["heap"]     = ESP.getFreeHeap();
    doc["time"]    = String(gpsData.year) + "-" +
                      (gpsData.month < 10 ? "0" : "") + String(gpsData.month) + "-" +
                      (gpsData.day   < 10 ? "0" : "") + String(gpsData.day)   + " " +
                      (gpsData.hour   < 10 ? "0" : "") + String(gpsData.hour)   + ":" +
                      (gpsData.minute < 10 ? "0" : "") + String(gpsData.minute) + ":" +
                      (gpsData.second < 10 ? "0" : "") + String(gpsData.second) + "Z";

    String out;
    serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

void setupWebServer() {
    if (!wifiConnected) return;
    webServer.on("/", handleRoot);
    webServer.on("/status.json", handleStatusJson);
    webServer.begin();
    Serial.printf("[WEB] Dashboard ready — http://%s/\n", WiFi.localIP().toString().c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
//  WiFi — WiFiManager with timeout
// ═══════════════════════════════════════════════════════════════════════════

void setupWiFi() {
    WiFiManager wm;

    // Silent — no debug output to serial
    wm.setDebugOutput(false);

    // If the portal is triggered, time out after 60s and continue without WiFi
    // so the N2K bus is never blocked waiting for credentials
    wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT);

    // Custom AP name and no password so it's easy to find on a phone
    bool connected = wm.autoConnect(WIFI_AP_NAME);

    if (connected) {
        wifiConnected = true;
        Serial.printf("[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        wifiConnected = false;
        Serial.println("[WiFi] Not connected — N2K running without OTA.");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  OTA — only started if WiFi connected
// ═══════════════════════════════════════════════════════════════════════════

void setupOTA() {
    if (!wifiConnected) return;
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.onStart([]()  { Serial.println("[OTA] Starting..."); });
    ArduinoOTA.onEnd([]()    { Serial.println("\n[OTA] Done. Rebooting."); });
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
        N2K_DEVICE_SERIAL_CODE, 100,
        N2K_DEVICE_MODEL_ID,
        N2K_DEVICE_SW_VERSION,
        N2K_DEVICE_MODEL_VERSION
    );
    NMEA2000.SetDeviceInformation(
        N2K_UNIQUE_NUMBER,
        N2K_DEVICE_FUNCTION,
        N2K_DEVICE_CLASS,
        N2K_MANUFACTURER_CODE
    );
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, N2K_ADDRESS);
    NMEA2000.ExtendTransmitMessages(TX_PGN_LIST);
    NMEA2000.EnableForward(false);
    NMEA2000.Open();
    Serial.println("[N2K] Bus open — address claiming started.");
}

// ═══════════════════════════════════════════════════════════════════════════
//  GPS init
// ═══════════════════════════════════════════════════════════════════════════

void setupGPS() {
    Serial1.begin(GPS_BAUD_TARGET, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
    delay(100);  // let UART settle

    uint8_t retries = 5;
    while (!gps.begin(Serial1, 1100, true) && retries > 0) {
        Serial.println("[GPS] Handshake retry...");
        delay(500);
        retries--;
    }

    if (retries == 0) {
        Serial.println("[GPS] *** Not detected — check JST-GH wiring and 5V supply ***");
        while (true) { ArduinoOTA.handle(); webServer.handleClient(); delay(10); }
    }

    gps.setUART1Output(COM_TYPE_UBX);
    gps.setNavigationFrequency(GPS_UPDATE_HZ);
    gps.setDynamicModel(DYN_MODEL_SEA);
    gps.setAutoPVTcallbackPtr(&onGpsData);   // register AFTER begin() succeeds
    gps.setAutoDOPcallbackPtr(&onGpsDop);    // HDOP/PDOP for Axiom accuracy estimate
    gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    Serial.printf("[GPS] HGLRC M100 Pro (u-blox M10) ready — %d Hz, SEA model\n", GPS_UPDATE_HZ);
}

// ═══════════════════════════════════════════════════════════════════════════
//  setup() / loop()
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n══════════════════════════════════════");
    Serial.println(" ESP32-S3 NMEA 2000 GPS Node  v1.0.0");
    Serial.println(" XH-S3E + HGLRC M100 Pro + SN65HVD230");
    Serial.println("══════════════════════════════════════");

    setupWiFi();
    setupOTA();
    setupMqtt();
    setupWebServer();
    setupNMEA2000();
    setupGPS();

    Serial.println("[BOOT] GPS ready. Waiting for fix...");
}

void loop() {
    if (wifiConnected) {
        ArduinoOTA.handle();
        mqttClient.loop();
        webServer.handleClient();
    }
    gps.checkUblox();
    gps.checkCallbacks();
    NMEA2000.ParseMessages();
    if (!gpsData.valid) return;
    uint32_t now = millis();
    if (now - lastPos     >= INTERVAL_PGN_129025) { lastPos     = now; sendPGN129025(); }
    if (now - lastCogSog  >= INTERVAL_PGN_129026) { lastCogSog  = now; sendPGN129026(); }
    if (now - lastGnss    >= INTERVAL_PGN_129029) { lastGnss    = now; sendPGN129029(); }
    if (now - lastSysTime >= INTERVAL_PGN_126992) { lastSysTime = now; sendPGN126992(); }
    if (now - lastMqtt    >= INTERVAL_MQTT_MS)    { lastMqtt    = now; publishMqtt();   }
}