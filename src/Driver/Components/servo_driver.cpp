#include "servo_driver.h"
#include "../../../Inc/pins.h"

// ===================== SERVO CONTROL =====================
Servo steering;

void initServo() {
  steering.attach(SERVO_PIN);
  steering.write(90);
}

void setServoAngle(int8_t angle) {
  steering.write(angle);
}