# ESP32 Line-Following Robot with Web Dashboard

This is an enhanced version of the line-following robot project that includes Wi-Fi connectivity and a web-based dashboard for real-time monitoring and control.

## Features

- All original line-following robot functionality preserved
- Wi-Fi connectivity with web server
- Real-time dashboard with sensor visualization
- WebSocket-based communication for low-latency updates
- Remote control via web interface
- Telemetry data display (speed, battery, state, obstacle detection)

## Hardware Requirements

- ESP32 WROOM-32UE development board
- Line-following robot with TCRT5000 sensors, motors, servo, and encoders
- Same hardware setup as the original project

## Setup Instructions

1. Update the Wi-Fi credentials in `src/middleware/network_handler.cpp`:
   - Replace `YOUR_WIFI_SSID` with your Wi-Fi network name
   - Replace `YOUR_WIFI_PASSWORD` with your Wi-Fi password

2. Build and upload the project to your ESP32 using PlatformIO

3. After successful upload, check the Serial Monitor for the ESP32's IP address

4. Open a web browser and navigate to the IP address to access the dashboard

## Dashboard Features

- Real-time sensor array visualization
- Telemetry data display (speed, battery, state, obstacle detection)
- Remote control buttons (start, stop, forward, left, right)
- Emergency stop button
- Connection status indicator

## Library Dependencies

- ESP32Servo
- ESPAsyncWebServer
- AsyncTCP
- ArduinoJson

## Troubleshooting

- If the page doesn't load, check that you're using the correct IP address
- If controls don't work, check WebSocket connection in browser console (F12)
- If sensors don't update, check the Serial Monitor for connection status
- Make sure firewall settings allow WebSocket connections