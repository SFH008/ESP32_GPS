#include "GPSParser.h"

// ═══════════════════════════════════════════════════════════════════════════
//  GPSParser.cpp — SparkFun u-blox GNSS v3
//
//  Notes:
//  - HDOP/PDOP are embedded in NAV-PVT; no separate setAutoNAVDOP needed.
//    (setAutoNAVODO exists but is the odometer message, not DOP.)
//  - getPVT(0): maxWait=0 makes the call non-blocking.
//  - Config saved to flash only for IOPORT (baud rate) — everything else
//    is re-applied on boot from RAM/BBR.
// ═══════════════════════════════════════════════════════════════════════════

bool GPSParser::begin() {
    // Try factory-default baud first
    _serial.begin(GPS_BAUD_INITIAL, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(250);

    if (!_gnss.begin(_serial)) {
        // Module may already be at target baud from a previous boot
        Serial.println("[GPS] No response at initial baud — trying target baud...");
        _serial.end();
        delay(50);
        _serial.begin(GPS_BAUD_TARGET, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        delay(250);
        if (!_gnss.begin(_serial)) {
            Serial.println("[GPS] ERROR: No response from NEO-M9N. Check G1/G2 wiring.");
            return false;
        }
        Serial.println("[GPS] Detected at target baud (previously configured).");
        return _configureModule(false);     // Skip baud change — already done
    }

    Serial.println("[GPS] Detected at initial baud.");
    return _configureModule(true);
}

bool GPSParser::_configureModule(bool changeBaud) {
    if (changeBaud && GPS_BAUD_TARGET != GPS_BAUD_INITIAL) {
        // Tell module to switch baud, then follow with our UART
        _gnss.setSerialRate(GPS_BAUD_TARGET);
        delay(100);
        _serial.end();
        delay(50);
        _serial.begin(GPS_BAUD_TARGET, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        delay(250);

        if (!_gnss.begin(_serial)) {
            // Failed — fall back to initial baud
            Serial.println("[GPS] Baud change failed — reverting to initial baud.");
            _serial.end();
            delay(50);
            _serial.begin(GPS_BAUD_INITIAL, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
            delay(250);
            _gnss.begin(_serial);
        } else {
            Serial.printf("[GPS] Baud changed to %d\n", GPS_BAUD_TARGET);
        }
    }

    // Output: UBX binary only — disable all NMEA output on UART1
    _gnss.setUART1Output(COM_TYPE_UBX);

    // Measurement rate: 100ms = 10 Hz
    _gnss.setMeasurementRate(100);
    _gnss.setNavigationRate(1);

    // Enable auto NAV-PVT — this gives us position, velocity, time AND
    // HDOP/PDOP all in one message. implicitUpdate=false means we call
    // getPVT() manually rather than the library draining UART automatically.
    _gnss.setAutoPVT(true, false, VAL_LAYER_RAM_BBR);

    // Dynamic model: SEA — tunes Kalman filter for vessel motion
    _gnss.setDynamicModel(DYN_MODEL_SEA);

    // Save UART baud rate to module flash so it survives cold power cycles.
    // Navigation config (rate, model) is cheap to re-apply each boot.
    _gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

    Serial.println("[GPS] Configured: 10Hz, UBX-only, SEA model, 4-constellation.");
    return true;
}

bool GPSParser::poll() {
    // getPVT(0): non-blocking — returns true only when a complete new
    // NAV-PVT packet has been received and decoded from the UART buffer.
    if (_gnss.getPVT(0)) {
        _updateData();
        return true;
    }
    return false;
}

void GPSParser::_updateData() {
    // ── Position ──────────────────────────────────────────────────────────
    _data.latitude    = (double)_gnss.getLatitude()    / 1e7;   // 1e-7 deg
    _data.longitude   = (double)_gnss.getLongitude()   / 1e7;
    _data.altitude    = (double)_gnss.getAltitudeMSL() / 1000.0; // mm → m
    _data.altitudeHAE = (double)_gnss.getAltitude()    / 1000.0; // mm → m

    // ── Motion ────────────────────────────────────────────────────────────
    // getHeading() = heading of motion in 1e-5 degrees, signed int32
    double h = (double)_gnss.getHeading() / 1e5;
    if (h < 0.0)    h += 360.0;
    if (h >= 360.0) h -= 360.0;
    _data.cogTrue = h;
    _data.sogMs   = (double)_gnss.getGroundSpeed() / 1000.0; // mm/s → m/s

    // ── Fix quality ───────────────────────────────────────────────────────
    _data.fixType   = _gnss.getFixType();
    _data.satellites= _gnss.getSIV();
    // HDOP/PDOP embedded in NAV-PVT; returned in hundredths
    _data.hdop = (double)_gnss.getHorizontalDOP() / 100.0;
    _data.pdop = (double)_gnss.getPDOP()           / 100.0;

    // ── Time (UTC) ────────────────────────────────────────────────────────
    _data.year   = _gnss.getYear();
    _data.month  = _gnss.getMonth();
    _data.day    = _gnss.getDay();
    _data.hour   = _gnss.getHour();
    _data.minute = _gnss.getMinute();
    _data.second = _gnss.getSecond();
    _data.nanos  = _gnss.getNanosecond();

    // ── Validity flags ────────────────────────────────────────────────────
    bool ok             = _gnss.getGnssFixOk();
    _data.positionValid = ok
                       && (_data.fixType >= 2)
                       && (_data.satellites >= MIN_SATELLITES_FOR_VALID);
    _data.timeValid     = _gnss.getTimeFullyResolved() && _gnss.getTimeValid();
    _data.cogSogValid   = ok && (_data.fixType >= 2);

    // Sequence ID: 0–252 (253–255 reserved by N2K spec)
    _data.sid = (_data.sid >= 252) ? 0 : _data.sid + 1;
}
