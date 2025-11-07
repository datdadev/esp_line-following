#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <ESP32Servo.h>
#include "error_codes.h"

// ===================== SERVO CONTROL =====================
extern Servo steering;

int8_t initServo();  // Returns error code
int8_t setServoAngle(int8_t angle);  // Returns error code
int8_t getCurrentServoAngle(int8_t* angle);  // Returns error code, angle via pointer

#endif // SERVO_DRIVER_H