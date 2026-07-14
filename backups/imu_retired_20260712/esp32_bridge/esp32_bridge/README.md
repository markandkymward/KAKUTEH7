ESP32-S3 UART to Wi-Fi bridge for KAKUTEH7 telemetry.

Features:
- Wi-Fi STA mode (joins your local network) when credentials are configured
- Automatic fallback to AP mode SSID: FC_TELEM if STA connection fails
- TCP server on port 5761
- Transparent binary bridge between FC UART and TCP client
- Ring-buffered FC->Laptop and Laptop->FC paths
- USB serial status output for bridge health counters

Default UART wiring (3.3V logic):
- FC telemetry UART TX -> ESP32-S3 RX (GPIO16)
- FC telemetry UART RX -> ESP32-S3 TX (GPIO17)
- FC GND -> ESP32-S3 GND

Build and flash with PlatformIO from this folder.

Local network setup (STA mode):
1. Edit `platformio.ini` build flags:
	- `BRIDGE_WIFI_STA_SSID`
	- `BRIDGE_WIFI_STA_PASS`
2. Flash firmware.
3. Open serial monitor at 115200 and read the assigned IP from logs:
	- `[bridge] STA connected: IP=...`
4. Connect your laptop to the same Wi-Fi and use that IP on TCP port 5761.

Fallback behavior:
- If STA credentials are empty or connection fails, bridge starts AP mode as `FC_TELEM`.

Serial status fields:
- TCP client connected/disconnected events
- bytes FC->Laptop
- bytes Laptop->FC
- UART overflow count
- TCP write failure count
