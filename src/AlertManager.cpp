// ═══════════════════════════════════════════════════════════════════════════
//  AlertManager.cpp — N2K PGN 126983 receive, siren drive, silence/ACK
// ═══════════════════════════════════════════════════════════════════════════

#include "AlertManager.h"
#include <ArduinoJson.h>

AlertManager::AlertManager(NMEA2000_esp32_twai &n2k, PubSubClient &mqtt)
    : _n2k(n2k), _mqtt(mqtt) {}

void AlertManager::begin() {
    pinMode(SIREN_PIN,        OUTPUT);
    pinMode(SIREN_BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(SIREN_PIN, LOW);
    Serial.println("[ALERT] AlertManager ready — siren GPIO " +
                   String((int)SIREN_PIN) + ", button GPIO " +
                   String((int)SIREN_BUTTON_PIN));
}

void AlertManager::update() {
    uint32_t now = millis();

    // Button debounce
    if (now - _lastButtonRead >= SIREN_BUTTON_DEBOUNCE) {
        _lastButtonRead = now;
        bool currentState = digitalRead(SIREN_BUTTON_PIN);
        if (_lastButtonState == HIGH && currentState == LOW) {
            if (_sirenOn) {
                Serial.println("[ALERT] Button pressed — silencing siren.");
                _sendAlertResponse(N2K_ALERT_RESP_SILENCE);
                _sirenOff_();
                _publishMqttStatus();
            }
        }
        _lastButtonState = currentState;
    }

    // Safety timeout
    if (_sirenOn && (now - _alert.startedAt >= SIREN_TIMEOUT_MS)) {
        Serial.println("[ALERT] Safety timeout — siren auto-off.");
        _sirenOff_();
    }

    // MQTT status 1 Hz while alert active
    if (_alert.active || _sirenOn) {
        if (now - _lastMqttPub >= 1000) {
            _lastMqttPub = now;
            _publishMqttStatus();
        }
    }
}

void AlertManager::handlePGN126983(const tN2kMsg &msg) {
    // PGN 126983 raw byte layout:
    //   [0]   bits 3:0 = Alert Type  (0=Emergency, 1=Alarm, 2=Warning, 3=Caution)
    //   [1]   Alert System
    //   [2]   Alert Sub-System
    //   [3-4] Alert ID (uint16 LE)
    //   [5]   Data Source NW ID
    //   [6]   Data Source Instance
    //   [7]   bits 3:0 = Alert State (0=Emergency/Active, 1=Active, 2=AwaitAck,
    //                                 3=Ack, 4=Silenced, 5=Acknowledged, 6=Normal)
    if (msg.DataLen < 8) return;

    uint8_t  alertType  = msg.Data[0] & 0x0F;
    uint8_t  alertState = msg.Data[7] & 0x0F;
    uint8_t  alertSys   = msg.Data[1];
    uint16_t alertID    = (uint16_t)msg.Data[3] | ((uint16_t)msg.Data[4] << 8);
    uint8_t  srcAddr    = msg.Source;

    bool isEmergency = (alertType == 0);
    bool isActive    = (alertState <= 2);
    bool isCleared   = (alertState >= 5);

    if (isEmergency && isActive) {
        _alert.alertID     = alertID;
        _alert.srcAddr     = srcAddr;
        _alert.alertSystem = alertSys;
        _alert.active      = true;
        if (!_sirenOn) {
            _alert.startedAt = millis();
            Serial.printf("[ALERT] Emergency! ID=%u src=0x%02X sys=%u — SIREN ON\n",
                          alertID, srcAddr, alertSys);
        }
        _sirenOn_();
        _publishMqttStatus();

    } else if (isEmergency && isCleared
               && _alert.active
               && alertID == _alert.alertID
               && srcAddr == _alert.srcAddr) {
        Serial.printf("[ALERT] Emergency ID=%u cleared — siren off.\n", alertID);
        _alert.active = false;
        _sirenOff_();
        _publishMqttStatus();
    }
}

void AlertManager::mqttSilence() {
    if (!_sirenOn) return;
    Serial.println("[ALERT] MQTT silence.");
    _sendAlertResponse(N2K_ALERT_RESP_SILENCE);
    _sirenOff_();
    _publishMqttStatus();
}

void AlertManager::mqttAck() {
    if (!_alert.active) return;
    Serial.println("[ALERT] MQTT ACK.");
    _sendAlertResponse(N2K_ALERT_RESP_ACKNOWLEDGE);
    _sirenOff_();
    _alert.active = false;
    _publishMqttStatus();
}

void AlertManager::_sirenOn_() {
    if (_sirenOn) return;
    _sirenOn = true;
    digitalWrite(SIREN_PIN, HIGH);
}

void AlertManager::_sirenOff_() {
    _sirenOn = false;
    digitalWrite(SIREN_PIN, LOW);
}

void AlertManager::_sendAlertResponse(uint8_t resp) {
    if (!_alert.active) return;
    tN2kMsg N2kMsg;
    N2kMsg.SetPGN(126984L);
    N2kMsg.Priority    = 7;
    N2kMsg.Destination = _alert.srcAddr;
    N2kMsg.AddByte(0x00);
    N2kMsg.AddByte(_alert.alertSystem);
    N2kMsg.Add2ByteUInt(_alert.alertID);
    N2kMsg.AddByte(resp);
    N2kMsg.AddByte(_alert.srcAddr);
    _n2k.SendMsg(N2kMsg);
    Serial.printf("[ALERT] PGN 126984 sent resp=0x%02X to 0x%02X alertID=%u\n",
                  resp, _alert.srcAddr, _alert.alertID);
}

void AlertManager::_publishMqttStatus() {
    if (!_mqtt.connected()) return;
    JsonDocument doc;
    doc["active"]  = _alert.active;
    doc["siren"]   = _sirenOn;
    doc["alertID"] = _alert.alertID;
    doc["srcAddr"] = _alert.srcAddr;
    doc["since_s"] = alertAgeSec();
    char buf[128];
    size_t n = serializeJson(doc, buf);
    _mqtt.publish(MQTT_TOPIC_ALERT_STATE, buf, n);
}