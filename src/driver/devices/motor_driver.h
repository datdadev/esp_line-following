#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "error_codes.h"

// ===================== MOTOR CONTROL =====================
int8_t setMotor(int16_t pwm);  // Returns error code (deprecated - use setMotorSpeed)
int8_t setMotorSpeed(float speed_mps); // Set motor speed in m/s using PID control
int8_t hardBrake();  // Returns error code
int8_t getMotor(int16_t* pwm); // Returns error code, current pwm via pointer
float getTargetSpeed(); // Get the current target speed in m/s
float getCurrentSpeed(); // Get the current speed in m/s
void updateCurrentSpeed(float speed); // Update current speed for PID controller

#endif // MOTOR_DRIVER_H