#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "error_codes.h"

// ===================== MOTOR CONTROL =====================
int8_t setMotor(int16_t pwm);  // Returns error code
int8_t hardBrake();  // Returns error code
int8_t getMotor(int16_t* pwm); // Returns error code, current pwm via pointer

#endif // MOTOR_DRIVER_H