# ESP32 Line-Following Robot

## Project Structure

This project follows a structured organization pattern based on embedded systems best practices:

```
Inc/                     # Public header files
Config/                  # System configuration (pin maps, parameters)
Docs/                    # Documentation, diagrams, and README files
Src/
├── App/                 # Application layer – main program logic
│   ├── main.cpp         # Main application
│   ├── motor_ctrl_test.cpp # Motor control test
│   ├── soft_start.h     # Soft start implementation header
│   └── soft_start.cpp   # Soft start implementation
│
├── BSP/                 # Board Support Package – board hardware initialization
│
├── Driver/              # Hardware drivers (low-level)
│   ├── Components/      # External modules and sensors
│   │   ├── sensor_driver.h
│   │   ├── sensor_driver.cpp
│   │   ├── servo_driver.h
│   │   └── servo_driver.cpp
│   └── Devices/         # Integrated devices and MCU peripherals
│       ├── motor_driver.h
│       └── motor_driver.cpp
│
└── MiddleWare/          # Middleware layer – RTOS, protocols, and libraries
    ├── pid_controller.h
    └── pid_controller.cpp
```

## Logic and State Machines

For detailed information about the pseudo-simplified logic and state machine implementation, see [Logic_and_State_Machines.md](Logic_and_State_Machines.md).

## Configuration

- **System Parameters**: Config/system_config.h
- **Pin Definitions**: Inc/pins.h

## Building

To build the project, use PlatformIO:

```bash
pio run
```

## Environments

- `esp32dev`: Main line-following application
- `motor_test`: Motor control and encoder testing