# ESP32-S3 GPS Node — Alert Siren & WiFi Config

## Hardware additions (this branch)

| Component | Connection |
|-----------|------------|
| Active 12V piezo buzzer | MOSFET drain |
| IRLZ44N MOSFET gate | GPIO 8 via 100Ω resistor |
| MOSFET source | GND |
| Flyback diode | Across buzzer terminals, cathode to 12V |
| Silence button (momentary → GND) | GPIO 9 (internal pull-up) |

## What was added

### N2K Alert handling (PGN 126983)
- Listens for Emergency Alarm alerts on the NMEA 2000 backbone
- Fires 12V siren on any `AlertType=Emergency, AlertState=Active`
- Sends PGN 126984 (Alert Response) back to the alerting device on silence/ACK
- Safety auto-off after 120s (`SIREN_TIMEOUT_MS` in `Config.h`)
- Note: `tN2kAlertResponse` not in ttlappalainen v4.x — raw byte constants used instead

### Silence / ACK paths
| Method | Action |
|--------|--------|
| Button press (GPIO 9) | Immediate siren off + PGN 126984 Silence |
| MQTT `gps-n2k/alert/silence` | Siren off + PGN 126984 Silence |
| MQTT `gps-n2k/alert/ack` | Siren off + PGN 126984 Acknowledge + clears alert |
| Source sends State=Normal | Siren off via N2K (no 126984 sent) |
| 120s timeout | Local siren off only — source still considers alert active |

### MQTT topics
| Topic | Direction | Purpose |
|-------|-----------|---------|
| `gps-n2k/alert/active` | Publish 1Hz | Alert status while active |
| `gps-n2k/alert/silence` | Subscribe | Silence siren |
| `gps-n2k/alert/ack` | Subscribe | ACK and clear alert |
| `gps-n2k/config/portal` | Subscribe | Reboot into WiFi config portal |

### Static IP via WiFiManager portal
- IP stored in NVS (`Preferences`, namespace `gps_cfg`)
- To reconfigure from Node-RED:
- Device reboots, `GPS-N2K-Setup` AP appears
- Connect phone, navigate to `192.168.2.4`
- Enter new Static IP, save — device reconnects on new IP
- Leave IP field blank to fall back to DHCP

### Current fixed IP
`192.168.2.144` — also reserve this via DHCP MAC reservation on RUTX12 as backup.

## Web dashboard
`http://192.168.2.144/` — live GPS data, map, alert banner with Silence/ACK buttons

## OTA
## Key library note
Alert PGNs 126983/126984 are not in ttlappalainen NMEA2000-library v4.x.
Raw byte parsing is used throughout `AlertManager.cpp`. If the library gains
support in a future version, `N2K_ALERT_RESP_SILENCE` / `N2K_ALERT_RESP_ACKNOWLEDGE`
in `AlertManager.h` can be replaced with the official enums.
