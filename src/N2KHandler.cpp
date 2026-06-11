#include "N2KHandler.h"

// ═══════════════════════════════════════════════════════════════════════════
//  N2KHandler.cpp
//
//  API notes confirmed from ttlappalainen/NMEA2000 source:
//
//  SetN2kGNSS() / SetN2kPGN129029() signature:
//    (msg, SID, DaysSince1970, SecondsSinceMidnight,
//     Lat, Lon, Altitude,
//     GNSStype, GNSSmethod,
//     nSatellites, HDOP, PDOP=0, GeoidalSeparation=0,
//     nReferenceStations=0, ReferenceStationType=N2kGNSSt_GPS,
//     ReferenceStationID=0, AgeOfCorrection=0)
//  No integrity parameter — the library hardcodes it internally.
//
//  System time source enum: N2ktimes_GPS  (NOT N2kts_GPSTime)
//  Defined in N2kTypes.h as: enum tN2kTimeSource { N2ktimes_GPS=0, ... }
// ═══════════════════════════════════════════════════════════════════════════

bool N2KHandler::begin(tNMEA2000* nmea2000) {
    _nmea2000 = nmea2000;
    return (_nmea2000 != nullptr);
}

void N2KHandler::update(const GpsData& gps) {
    if (!_nmea2000) return;

    uint32_t now = millis();

    // ── Rapid PGNs: 10 Hz ─────────────────────────────────────────────────
    if (now - _lastRapid >= INTERVAL_RAPID_MS) {
        _lastRapid = now;
        if (gps.positionValid) _sendPositionRapid(gps);
        if (gps.cogSogValid)   _sendCogSogRapid(gps);
    }

    // ── Full fix + time: 1 Hz ─────────────────────────────────────────────
    if (now - _lastFull >= INTERVAL_FULL_FIX_MS) {
        _lastFull = now;
        if (gps.positionValid) _sendGnssPosition(gps);
        if (gps.timeValid)     _sendSystemTime(gps);
    }

    // ParseMessages() must be called every loop to service address claiming
    // and respond to group function requests from other N2K devices.
    _nmea2000->ParseMessages();
}

// ─── PGN 129025 — Position, Rapid Update ────────────────────────────────────

void N2KHandler::_sendPositionRapid(const GpsData& gps) {
    tN2kMsg msg;
    SetN2kLatLonRapid(msg, gps.latitude, gps.longitude);
    _nmea2000->SendMsg(msg);
}

// ─── PGN 129026 — COG & SOG, Rapid Update ───────────────────────────────────

void N2KHandler::_sendCogSogRapid(const GpsData& gps) {
    tN2kMsg msg;
    SetN2kCOGSOGRapid(msg,
        gps.sid,
        N2khr_true,
        DegToRad(gps.cogTrue),
        gps.sogMs
    );
    _nmea2000->SendMsg(msg);
}

// ─── PGN 129029 — GNSS Position Data ────────────────────────────────────────

void N2KHandler::_sendGnssPosition(const GpsData& gps) {
    tN2kMsg msg;

    uint16_t days = _daysSince1970(gps.year, gps.month, gps.day);
    double   secs = _secondsSinceMidnight(gps.hour, gps.minute, gps.second, gps.nanos);

    // Map u-blox fixType to N2K GNSS type
    tN2kGNSStype gnssType;
    switch (gps.fixType) {
        case 4:  gnssType = N2kGNSSt_GPSSBASWAASGLONASS; break; // GNSS+DR
        default: gnssType = N2kGNSSt_GPS;                break; // Standard 3D
    }

    // Map u-blox fixType to N2K GNSS method
    tN2kGNSSmethod gnssMethod = (gps.fixType >= 2) ? N2kGNSSm_GNSSfix : N2kGNSSm_noGNSS;

    // SetN2kGNSS is the inline wrapper for SetN2kPGN129029
    SetN2kGNSS(msg,
        gps.sid,
        days,
        secs,
        gps.latitude,
        gps.longitude,
        gps.altitudeHAE,    // N2K spec: height above ellipsoid, not MSL
        gnssType,
        gnssMethod,
        gps.satellites,
        gps.hdop,
        gps.pdop,
        0.0                 // GeoidalSeparation: 0 if not available
        // nReferenceStations defaults to 0 (no DGPS)
    );
    _nmea2000->SendMsg(msg);
}

// ─── PGN 126992 — System Time ────────────────────────────────────────────────

void N2KHandler::_sendSystemTime(const GpsData& gps) {
    tN2kMsg msg;

    uint16_t days = _daysSince1970(gps.year, gps.month, gps.day);
    double   secs = _secondsSinceMidnight(gps.hour, gps.minute, gps.second, gps.nanos);

    // tN2kTimeSource enum from N2kTypes.h: N2ktimes_GPS=0
    SetN2kSystemTime(msg, gps.sid, days, secs, N2ktimes_GPS);
    _nmea2000->SendMsg(msg);
}

// ─── Time helpers ────────────────────────────────────────────────────────────

uint16_t N2KHandler::_daysSince1970(uint16_t year, uint8_t month, uint8_t day) {
    // Proleptic Gregorian calendar → Julian Day Number → offset from 1970-01-01
    int y = year, m = month, d = day;
    if (m < 3) { y--; m += 12; }
    int A   = y / 100;
    int B   = 2 - A + A / 4;
    long jdn = (long)(365.25  * (y + 4716))
             + (long)(30.6001 * (m + 1))
             + d + B - 1524;
    return (uint16_t)(jdn - 2440588L); // JDN of 1970-01-01 = 2440588
}

double N2KHandler::_secondsSinceMidnight(uint8_t h, uint8_t m, uint8_t s, uint32_t ns) {
    return (double)h * 3600.0
         + (double)m * 60.0
         + (double)s
         + (double)ns / 1e9;
}
