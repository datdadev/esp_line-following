#include <Arduino.h>
#include "pins.h"
#include "motor_driver.h"
#include "error_codes.h"

// ===================== MOTOR CONTROL =====================
static int16_t currentMotorPWM = 0;  // Track current motor PWM value

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
  

  
  // Return success (could be extended to check for actual motor response)
  return ERROR_SUCCESS;
}

// Hard brake function - stops motor quickly by shorting motor terminals
int8_t hardBrake() {
  // Check if pins are properly configured (if needed in future extensions)
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  currentMotorPWM = 0;  // Set the current PWM to 0 when braking
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