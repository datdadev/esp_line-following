#include <Arduino.h>
#include "pins.h"
#include "motor_driver.h"
#include "error_codes.h"
#include "system_config.h"
#include "../middleware/pid_controller.h"
#include <algorithm>

// ===================== MOTOR CONTROL =====================
static int16_t currentMotorPWM = 0;  // Track current motor PWM value
static float targetSpeed = 0.0;      // Target speed in m/s
static float currentSpeed = 0.0;     // Current speed in m/s
static PIDController speedPID = {0}; // PID controller for speed control - will be initialized in setMotorSpeed

int8_t setMotor(int16_t pwm) {
  // Constrain PWM value to valid range
  pwm = constrain(pwm, -255, 255);
  currentMotorPWM = pwm;
  
  // Check for pin initialization issues (if needed in future extensions)
  // For now, we assume pins are properly configured
  
  if (pwm >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm);
  }
  
  // Update target speed based on PWM (approximate conversion)
  targetSpeed = pwm * SPEED_PER_PWM;

  // Return success (could be extended to check for actual motor response)
  return ERROR_SUCCESS;
}

int8_t setMotorSpeed(float speed_mps) {
  // Initialize PID controller if not already initialized
  static bool pid_initialized = false;
  if (!pid_initialized) {
    speedPID = pid_init(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_MIN_OUTPUT, SPEED_PID_MAX_OUTPUT); // PID controller for speed control
    pid_initialized = true;
  }
  
  // Constrain speed to valid range
  float max_speed = MAX_SPEED;
  speed_mps = std::max(-max_speed, std::min(max_speed, speed_mps));
  targetSpeed = speed_mps;

  // Get current actual speed for PID calculation
  // This assumes that currentSpeed is updated from the main loop using encoder data
  float dt = Ts; // Use the system sampling time from system_config.h
  
  // Compute PID output (this is the PWM value needed)
  float pid_output = pid_compute(&speedPID, targetSpeed, currentSpeed, dt);
  int16_t pwm = static_cast<int16_t>(pid_output);
  
  // Constrain PWM to valid range
  pwm = constrain(pwm, 0, 255);
  currentMotorPWM = pwm;

  if (pwm >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm);
  }

  return ERROR_SUCCESS;
}

// Update the current speed used by the PID controller
void updateCurrentSpeed(float speed) {
  currentSpeed = speed;
}

float getTargetSpeed() {
    return targetSpeed;
}

float getCurrentSpeed() {
    return currentSpeed;
}

// Hard brake function - stops motor quickly by shorting motor terminals
int8_t hardBrake() {
  // Check if pins are properly configured (if needed in future extensions)
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  currentMotorPWM = 0;  // Set the current PWM to 0 when braking
  targetSpeed = 0.0;    // Set target speed to 0
  currentSpeed = 0.0;   // Set current speed to 0
  analogWrite(PWMA, 255);  // Apply maximum braking power

  return ERROR_SUCCESS;
}

int8_t getMotor(int16_t* pwm) {
  if (pwm == nullptr) {
    return ERROR_INVALID_PARAMETER;
  }
  
  *pwm = currentMotorPWM;
  return ERROR_SUCCESS;
}