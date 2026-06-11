Wire it up — SN65HVD230 to GPIO4/5, M100 Pro JST-GH to GPIO16/17 + 5V, STNG drop cable to transceiver
Fill in WiFi credentials in Config.h
First flash via USB: temporarily comment out upload_protocol = espota in platformio.ini, then pio run -t upload
Check serial monitor for [GPS] and [N2K] boot messages
Verify PGNs on the bus with candump can0 on the Pi


GPIO4, GPIO5 (TWAI) — listed as first-priority free GPIOs with no special functions or restrictions. Safe for TWAI TX/RX. Manuals+
GPIO16, GPIO17 (GPS UART) — also listed as first-priority free GPIOs. UART1 defaults to GPIO17 TX / GPIO18 RX on the ESP32-S3, but can be remapped via the GPIO Matrix — our code assigns Serial1 explicitly to GPIO16/17 which overrides the default, so no conflict. Manuals+Manuals+
Pins to avoid — GPIO26–32 are reserved for SPI flash and PSRAM, and on the XH-S3E N16R8 with Octal PSRAM, GPIO33–37 are also consumed by SPIIO4–SPIIO7 and SPIDQS. GPIO0, 3, 45, 46 are strapping pins to avoid, and GPIO19/20 are native USB. Pimoroni + 2
