#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// ===================== SYSTEM CONSTANTS =====================
constexpr float Ts = 0.01;             // Sampling time (10 ms)
constexpr uint8_t NUM_IR = 7;          // 7 TCRT5000 sensors
constexpr float Kp = 0.5;
constexpr float Ki = 0.0;
constexpr float Kd = 0.1;

constexpr uint8_t SONAR_TH_OBS = 45;   // cm
constexpr uint8_t JUNCTION_COUNT = 5;
constexpr float DIST_NEAR_GOAL = 35.0; // cm after junction
constexpr int16_t PWM_NORMAL = 200;
constexpr int16_t PWM_SLOW = 120;

// Soft-start configuration
constexpr uint32_t SOFT_START_DURATION = 1000;  // 1 second for soft start

// PID derivative filtering
constexpr float DERIVATIVE_FILTER_ALPHA = 0.2;  // Low-pass filter coefficient (0.0-1.0, lower = more filtering)

#endif // SYSTEM_CONFIG_H