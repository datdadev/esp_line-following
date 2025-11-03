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

- `Inc/` - Public header files
- `Config/` - System configuration files (parameters, pin mappings)
- `Docs/` - Documentation
- `Src/App/` - Application layer source code
- `Src/Driver/` - Hardware drivers
- `Src/MiddleWare/` - Middleware components (PID controller, etc.)
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
# Build the main application
pio run

# Build the motor test
pio run -e motor_test
```

## Configuration

System parameters can be adjusted in `Config/system_config.h`
Pin definitions are in `Inc/pins.h`