#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// ===================== SYSTEM CONSTANTS =====================
constexpr float Ts = 0.01;             // Sampling time (10 ms)
constexpr uint8_t NUM_IR = 7;          // 7 TCRT5000 sensors

// PID Controller Parameters (for backward compatibility)
constexpr float Kp = 0.5;
constexpr float Ki = 0.0;
constexpr float Kd = 0.1;

// Lyapunov Controller Parameters
constexpr double LYP_K1 = 1.5;         // Proportional gain for arctan(e) - helps with stability
constexpr double LYP_K2 = 0.8;         // Proportional gain for e - main steering response
constexpr double LYP_K3 = 0.15;        // Derivative gain for de/dt - reduces oscillation
constexpr double LYP_STEERING_LIMIT = 40.0;  // Maximum steering angle offset (degrees)

constexpr uint16_t SONAR_TH_OBS = 450;   // mm
constexpr uint8_t JUNCTION_COUNT = 5;
constexpr float DIST_NEAR_GOAL = 35.0; // cm after junction

// Motor and Drive Configuration
constexpr float GEAR_RATIO = 10.0;     // Gear reduction ratio (motor:wheel)
constexpr float WHEEL_DIAMETER = 0.066; // Wheel diameter in meters (66mm)
constexpr int PULSES_PER_REV = 11;     // Encoder pulses per motor revolution (PPR)
constexpr float QUAD_FACTOR = 4.0;     // Quadrature decoding factor (4x)
constexpr float COUNTS_PER_REV = PULSES_PER_REV * GEAR_RATIO * QUAD_FACTOR; // 440 counts/rev

// Speed Configuration (1 m/s corresponds to 255 PWM)
constexpr float MAX_SPEED = 1.0;       // Maximum speed in m/s
constexpr int16_t MAX_PWM = 255;       // Maximum PWM value for 1m/s
constexpr float SPEED_PER_PWM = MAX_SPEED / MAX_PWM;  // Speed per PWM unit (0.00392 m/s per PWM)

// Default PWM values (converted from speeds)
constexpr int16_t PWM_NORMAL = static_cast<int16_t>(1.0 / SPEED_PER_PWM); // ~255 for 1m/s
constexpr int16_t PWM_SLOW = static_cast<int16_t>(0.6 / SPEED_PER_PWM);   // ~153 for 0.6m/s

// Soft-start configuration
constexpr uint32_t SOFT_START_DURATION = 1000;  // 1 second for soft start

// PID derivative filtering
constexpr float DERIVATIVE_FILTER_ALPHA = 0.2;  // Low-pass filter coefficient (0.0-1.0, lower = more filtering)

#endif // SYSTEM_CONFIG_H