#pragma once

// ═══════════════════════════════════════════════════════════════════════════
//  AlertManager.h — N2K PGN 126983 receive, siren drive, silence/ACK
//
//  Alert PGNs (126983/126984) are NOT in ttlappalainen v4.19.0.
//  Response codes are defined here as raw byte constants per the N2K spec.
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <NMEA2000_esp32_twai.h>
#include <N2kMessages.h>
#include <PubSubClient.h>
#include "Config.h"

// ── PGN 126984 response command byte values (N2K spec table) ────────────────
#define N2K_ALERT_RESP_SILENCE      0x01
#define N2K_ALERT_RESP_ACKNOWLEDGE  0x02

// ── Active alert record ──────────────────────────────────────────────────────
struct ActiveAlert {
    uint16_t alertID     = 0;
    uint8_t  srcAddr     = 0;
    uint8_t  alertSystem = 0;
    bool     active      = false;
    uint32_t startedAt   = 0;
};

// ── AlertManager ─────────────────────────────────────────────────────────────
class AlertManager {
public:
    AlertManager(NMEA2000_esp32_twai &n2k, PubSubClient &mqtt);

    void begin();
    void update();
    void handlePGN126983(const tN2kMsg &msg);
    void mqttSilence();
    void mqttAck();

    bool     isSirenActive() const { return _sirenOn; }
    uint16_t lastAlertID()   const { return _alert.alertID; }
    uint8_t  lastAlertSrc()  const { return _alert.srcAddr; }
    uint32_t alertAgeSec()   const {
        return _alert.active ? (millis() - _alert.startedAt) / 1000 : 0;
    }

private:
    NMEA2000_esp32_twai &_n2k;
    PubSubClient        &_mqtt;

    ActiveAlert  _alert;
    bool         _sirenOn         = false;
    uint32_t     _lastMqttPub     = 0;
    uint32_t     _lastButtonRead  = 0;
    bool         _lastButtonState = true;   // HIGH = not pressed

    void _sirenOn_();
    void _sirenOff_();
    void _sendAlertResponse(uint8_t resp);  // raw byte, no lib enum needed
    void _publishMqttStatus();
};
