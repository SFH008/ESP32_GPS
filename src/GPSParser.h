#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "Config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  GPSParser.h
//  Wraps SparkFun u-blox GNSS v3 library over hardware UART.
//  SFE_UBLOX_GNSS_SERIAL is the correct class for UART in v3.
// ═══════════════════════════════════════════════════════════════════════════

struct GpsData {
    double   latitude       = 0.0;   // degrees
    double   longitude      = 0.0;   // degrees
    double   altitude       = 0.0;   // metres above MSL
    double   altitudeHAE    = 0.0;   // metres above ellipsoid (for PGN 129029)
    double   cogTrue        = 0.0;   // degrees true 0–360
    double   sogMs          = 0.0;   // m/s

    uint8_t  fixType        = 0;     // 0=none 2=2D 3=3D 4=GNSS+DR
    uint8_t  satellites     = 0;
    double   hdop           = 99.9;
    double   pdop           = 99.9;

    uint16_t year           = 0;
    uint8_t  month          = 0;
    uint8_t  day            = 0;
    uint8_t  hour           = 0;
    uint8_t  minute         = 0;
    uint8_t  second         = 0;
    uint32_t nanos          = 0;

    bool     positionValid  = false;
    bool     timeValid      = false;
    bool     cogSogValid    = false;
    uint8_t  sid            = 0;     // N2K Sequence ID 0–252
};

class GPSParser {
public:
    bool            begin();
    bool            poll();                         // true on new NAV-PVT packet
    const GpsData&  data() const { return _data; }

private:
    SFE_UBLOX_GNSS_SERIAL _gnss;                   // v3: UART class
    HardwareSerial        _serial{GPS_UART_PORT};
    GpsData               _data;

    bool _configureModule(bool changeBaud);
    void _updateData();
};
