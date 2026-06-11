#pragma once

#include <Arduino.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include "GPSParser.h"
#include "Config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  N2KHandler.h — PGN transmit handler
//
//  PGNs:
//    129025  Position Rapid Update         10 Hz
//    129026  COG & SOG Rapid Update        10 Hz
//    129029  GNSS Position Data             1 Hz
//    126992  System Time                    1 Hz
// ═══════════════════════════════════════════════════════════════════════════

class N2KHandler {
public:
    bool begin(tNMEA2000* nmea2000);
    void update(const GpsData& gps);   // Call every loop iteration

private:
    tNMEA2000* _nmea2000  = nullptr;
    uint32_t   _lastRapid = 0;
    uint32_t   _lastFull  = 0;

    void _sendPositionRapid(const GpsData& gps);    // PGN 129025
    void _sendCogSogRapid(const GpsData& gps);      // PGN 129026
    void _sendGnssPosition(const GpsData& gps);     // PGN 129029
    void _sendSystemTime(const GpsData& gps);       // PGN 126992

    static uint16_t _daysSince1970(uint16_t y, uint8_t m, uint8_t d);
    static double   _secondsSinceMidnight(uint8_t h, uint8_t m, uint8_t s, uint32_t ns);
};
