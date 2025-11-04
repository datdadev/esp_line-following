#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <ESP32Servo.h>

// ===================== SERVO CONTROL =====================
extern Servo steering;

void initServo();
void setServoAngle(int8_t angle);

#endif // SERVO_DRIVER_H