#include "motor_driver.h"
#include <Arduino.h>
#include "../../../Inc/pins.h"

// ===================== MOTOR CONTROL =====================
void setMotor(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm);
  }
}

// Hard brake function - stops motor quickly by shorting motor terminals
void hardBrake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);  // Apply maximum braking power
}