# ESP32 Line-Following Robot

A sophisticated line-following robot project built for ESP32 with advanced features including PID control, obstacle avoidance, junction detection, and WiFi-based remote monitoring.

## 🚀 Features

- PID-based line following algorithm with precise control
- Real-time monitoring via WiFi web dashboard
- Ultrasonic obstacle detection and avoidance
- Junction detection and navigation
- Soft-start motor control for smooth acceleration
- Encoder-based distance tracking and speed calculation
- Configurable parameters for different scenarios
- WebSocket-based real-time telemetry

## 📁 Project Structure

```
esp_line-following/
├── .env                          # WiFi credentials (gitignored)
├── platformio.ini               # PlatformIO build configuration
├── .gitignore                   # Git ignore rules
├── README.md                    # This file
├── archives/                    # Archived code and experiments
├── backup_structure/           # Backup files
├── config/                     # System configuration files
├── credentials/                # WiFi credentials (gitignored)
├── docs/                       # Documentation
├── inc/                        # Public header files
├── include/                    # Additional include files
├── lib/                        # Third-party libraries
├── others/                     # Other utility files
├── reqs/                       # Requirements
├── src/                        # Source code
│   ├── app/                    # Main application code
│   ├── bsp/                    # Board support package
│   ├── driver/                 # Hardware drivers
│   └── middleware/             # Middleware components
└── test/                       # Test files
```

## 🛠️ Hardware Components

- **Microcontroller**: ESP32 WROOM32UE
- **Line Sensors**: 7x TCRT5000 infrared sensors for line detection
- **Obstacle Detection**: HC-SR04 ultrasonic sensor
- **Motor Driver**: TB6612FNG dual motor driver
- **Steering**: Standard servo motor
- **Encoder**: Quadrature encoder for distance/speed measurement
- **User Interface**: Boot button for start control

## 📋 Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html) or 
- VSCode with [PlatformIO extension](https://platformio.org/install/ide?install=vscode)

## ⚙️ Setup and Configuration

### 1. WiFi Configuration

**Option A: Using Environment Variables (Recommended)**
1. Create a `.env` file in the project root:
   ```bash
   # .env
   YOUR_WIFI_SSID=your_network_name
   YOUR_WIFI_PASSWORD=your_network_password
   ```
2. The `.env` file will be automatically gitignored

**Option B: Using Credentials Header File**
1. Copy the example credentials file:
   ```bash
   cp credentials/wifi_credentials.h.example credentials/wifi_credentials.h
   ```
2. Edit `credentials/wifi_credentials.h` with your WiFi credentials:
   ```cpp
   #define WIFI_SSID "YourNetworkName"
   #define WIFI_PASSWORD "YourNetworkPassword"
   ```
3. The `credentials/` directory is gitignored to protect your information

### 2. PlatformIO Build

The project uses PlatformIO for building and uploading:

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Clean build artifacts
pio run --target clean
```

## 🌐 Web Dashboard

Once the ESP32 connects to WiFi, access the dashboard at:
```
http://[ESP32_IP_ADDRESS]
```

The dashboard features:
- Real-time sensor visualization
- Speed and battery monitoring
- Robot state display
- Remote control buttons
- WebSocket-based telemetry updates

## 🏁 Usage

1. Configure your WiFi credentials as described above
2. Build and upload the code to your ESP32
3. Power on the robot
4. Connect the robot to your WiFi network
5. Access the web dashboard from any device on the same network
6. Press the BOOT button on the ESP32 to start line following
7. Monitor and control the robot remotely

## 🔧 Configuration

System parameters can be adjusted in:
- `config/system_config.h` - Core system parameters
- `inc/pins.h` - Pin assignments
- `src/middleware/network_handler.cpp` - Network settings

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📚 Documentation

Additional documentation can be found in the `docs/` directory:
- [logic_and_state_machines.md](docs/logic_and_state_machines.md) - Detailed state machine implementation
- [include_readme.md](docs/include_readme.md) - Information about header files
- [lib_readme.md](docs/lib_readme.md) - Information about project libraries
- [test_readme.md](docs/test_readme.md) - Information about testing

## 🐛 Troubleshooting

- **Can't connect to WiFi**: Verify credentials in the configuration file
- **Dashboard not loading**: Check the IP address in the serial monitor
- **Robot not moving**: Verify motor connections and power supply
- **Sensors not responding**: Check TCRT5000 wiring and calibration