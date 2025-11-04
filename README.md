# ESP32 Line-Following Robot

A sophisticated line-following robot project built for ESP32 with advanced features including PID control, obstacle avoidance, and junction detection.

## Features

- PID-based line following algorithm
- Ultrasonic obstacle detection and avoidance
- Junction detection and navigation
- Soft-start motor control
- Encoder-based distance tracking
- Configurable parameters

## Project Structure

The project follows a structured organization:

- `inc/` - Public header files
- `config/` - System configuration files (parameters, pin mappings)
- `docs/` - Documentation
- `src/app/` - Application layer source code
- `src/driver/` - Hardware drivers
- `src/middleưare/` - Middleware components (PID controller, etc.)
- `platformio.ini` - PlatformIO build configuration

## Hardware Components

- ESP32 WROOM32UE
- 7x TCRT5000 infrared sensors for line detection
- HC-SR04 ultrasonic sensor for obstacle detection
- TB6612FNG motor driver
- Servo motor for steering
- Encoder for distance measurement

## Building

Prerequisites:
- PlatformIO Core or VSCode with PlatformIO extension

Build commands:
```bash
pio run
```

## Documentation

Additional documentation can be found in the `docs/` directory:
- [include_readme.md](docs/include_readme.md) - Information about header files
- [lib_readme.md](docs/lib_readme.md) - Information about project libraries
- [test_readme.md](docs/test_readme.md) - Information about testing
- [logic_and_state_machines.md](docs/logic_and_state_machines.md) - Detailed state machine implementation

## Configuration

System parameters can be adjusted in `config/system_config.h`
Pin definitions are in `inc/pins.h`