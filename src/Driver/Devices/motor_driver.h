#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

// ===================== MOTOR CONTROL =====================
void setMotor(int16_t pwm);
void hardBrake();

#endif // MOTOR_DRIVER_H