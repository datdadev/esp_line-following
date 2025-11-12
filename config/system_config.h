#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// ===================== SYSTEM CONSTANTS =====================
constexpr float Ts = 0.01;             // Sampling time (10 ms)
constexpr uint8_t NUM_IR = 7;          // 7 TCRT5000 sensors

// PID Controller Parameters (for backward compatibility)
constexpr float Kp = 0.5;
constexpr float Ki = 0.0;
constexpr float Kd = 0.1;

// Speed Controller PID Parameters
constexpr float SPEED_PID_KP = 255.0;    // Proportional gain for speed control
constexpr float SPEED_PID_KI = 0.0;    // Integral gain for speed control  
constexpr float SPEED_PID_KD = 0.0;    // Derivative gain for speed control

// Speed Controller Output Limits
constexpr float SPEED_PID_MIN_OUTPUT = 0.0;  // Minimum output for speed PID
constexpr float SPEED_PID_MAX_OUTPUT = 255.0;   // Maximum output for speed PID

// Lyapunov Controller Parameters
constexpr double LYP_K1 = 1.5;         // Proportional gain for arctan(e) - helps with stability
constexpr double LYP_K2 = 0.8;         // Proportional gain for e - main steering response
constexpr double LYP_K3 = 0.15;        // Derivative gain for de/dt - reduces oscillation
constexpr double LYP_STEERING_LIMIT = 40.0;  // Maximum steering angle offset (degrees)

constexpr uint16_t SONAR_TH_OBS = 450;   // mm
constexpr uint8_t JUNCTION_COUNT = 5;
constexpr float DIST_NEAR_GOAL = 35.0; // cm after junction

// Motor and Drive Configuration
constexpr float GEAR_RATIO = 10.0;      // Gear reduction ratio (motor:wheel)
constexpr float DIFF_RATIO = 2.714;     // Differential ratio
constexpr float WHEEL_DIAMETER = 0.065; // Wheel diameter in meters (65mm)
constexpr int PULSES_PER_REV = 11;      // Encoder pulses per motor revolution (PPR)
constexpr float QUAD_FACTOR = 4.0;      // Quadrature decoding factor (4x)
constexpr float COUNTS_PER_REV = PULSES_PER_REV * GEAR_RATIO * DIFF_RATIO * QUAD_FACTOR; // 440 counts/rev

// Speed Configuration (1.2 m/s corresponds to 255 PWM)
constexpr float MAX_SPEED = 1.2;       // Maximum speed in m/s
constexpr int16_t MAX_PWM = 255;       // Maximum PWM value for 1.2m/s
constexpr float SPEED_PER_PWM = MAX_SPEED / MAX_PWM;  // Speed per PWM unit (0.00392 m/s per PWM)

// Default PWM values (converted from speeds)
constexpr int16_t PWM_NORMAL = static_cast<int16_t>(1.0 / SPEED_PER_PWM); // ~255 for 1.2m/s
constexpr int16_t PWM_SLOW = static_cast<int16_t>(0.5 / SPEED_PER_PWM);

// Soft-start configuration
constexpr uint32_t SOFT_START_DURATION = 1000;  // 1 second for soft start

// Obstacle Avoidance Distances (in meters)
constexpr float OBSTACLE_AVOID_S1 = 264.28;
constexpr float OBSTACLE_AVOID_S2 = 528.56;
constexpr float OBSTACLE_AVOID_S3 = 264.28;

// Servo angles for obstacle avoidance - 23.336
constexpr int8_t SERVO_AVOID_LEFT = 66.664;
constexpr int8_t SERVO_AVOID_RIGHT = 113.336;  // Right turn servo angle during obstacle avoidance

// Acceleration parameters for smooth ramp control
constexpr float MOTOR_ACCELERATION = 0.5;   // Acceleration in m/s^2
constexpr float MOTOR_DECELERATION = 0.5;   // Deceleration in m/s^2 (can be different from acceleration)

// PID derivative filtering
constexpr float DERIVATIVE_FILTER_ALPHA = 0.2;  // Low-pass filter coefficient (0.0-1.0, lower = more filtering)

#endif // SYSTEM_CONFIG_H