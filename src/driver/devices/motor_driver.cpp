#include <Arduino.h>
#include "pins.h"
#include "motor_driver.h"
#include "error_codes.h"

// ===================== MOTOR CONTROL =====================
int8_t setMotor(int16_t pwm) {
  // Constrain PWM value to valid range
  pwm = constrain(pwm, -255, 255);
  
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
  
  #ifdef DEBUG_ENABLED
  Serial.print("Motor: Set PWM to ");
  Serial.println(pwm);
  #endif
  
  // Return success (could be extended to check for actual motor response)
  return ERROR_SUCCESS;
}

// Hard brake function - stops motor quickly by shorting motor terminals
int8_t hardBrake() {
  // Check if pins are properly configured (if needed in future extensions)
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);  // Apply maximum braking power
  
  #ifdef DEBUG_ENABLED
  Serial.println("Motor: Hard brake applied");
  #endif
  
  return ERROR_SUCCESS;
}