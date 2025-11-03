Inc/                     # Public header files
Config/                  # System configuration (pin maps, parameters)
Docs/                    # Documentation, diagrams, and README files
Build/                   # Build scripts and output files
Src/
├── App/                 # Application layer – main program logic
│   ├── main.c
│   ├── control/         # Control logic
│   ├── tasks/           # Task management
│   └── ui/              # User interface
│
├── BSP/                 # Board Support Package – board hardware initialization
│   ├── bsp_init.c
│   ├── bsp_led.c
│   └── bsp_uart.c
│
├── Driver/              # Hardware drivers (low-level)
│   ├── Components/      # External modules and sensors
│   │   ├── mpu6050/
│   │   │   ├── mpu6050.c
│   │   │   └── mpu6050.h
│   │   └── oled/
│   │       ├── ssd1306.c
│   │       └── ssd1306.h
│   │
│   └── Devices/         # Integrated devices and MCU peripherals
│       ├── tb6612/
│       │   ├── tb6612.c
│       │   └── tb6612.h
│       └── encoder/
│           ├── encoder.c
│           └── encoder.h
│
└── MiddleWare/          # Middleware layer – RTOS, protocols, and libraries
    ├── FreeRTOS/
    │   ├── freertos.c
    │   └── freertos_hooks.c
    └── Protocol/
        ├── modbus.c
        └── modbus.h
