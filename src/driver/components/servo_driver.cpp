#include "servo_driver.h"
#include "pins.h"
#include "error_codes.h"
#include <Arduino.h>
#include "error_codes.h"

// ===================== SERVO CONTROL =====================
Servo steering;

int8_t initServo() {
  if (steering.attached()) {
    #ifdef DEBUG_ENABLED
    Serial.println("Servo: Already initialized");
    #endif
    return ERROR_ALREADY_INITIALIZED;  // Servo already initialized
  }
  
  if (!steering.attach(SERVO_PIN)) {
    #ifdef DEBUG_ENABLED
    Serial.println("Servo: Failed to attach to pin");
    #endif
    return ERROR_SERVO_INIT_FAILED;  // Failed to attach servo
  }
  
  steering.write(90);  // Set initial position to center (90 degrees)
  #ifdef DEBUG_ENABLED
  Serial.println("Servo: Initial position set to center");
  #endif
  
  #ifdef DEBUG_ENABLED
  Serial.println("Servo: Initialization successful");
  #endif
  return ERROR_SUCCESS;
}

int8_t setServoAngle(int8_t angle) {
  if (!steering.attached()) {
    #ifdef DEBUG_ENABLED
    Serial.println("Servo: Not initialized before setting angle");
    #endif
    return ERROR_NOT_INITIALIZED;  // Servo not initialized
  }
  
  // Validate angle range (typically servos work between 0-180 degrees)
  if (angle < 0 || angle > 180) {
    #ifdef DEBUG_ENABLED
    Serial.print("Servo: Angle out of range: ");
    Serial.println(angle);
    #endif
    return ERROR_SERVO_OUT_OF_RANGE;  // Angle out of acceptable range
  }
  
  steering.write(angle);  // Set servo to requested angle
  #ifdef DEBUG_ENABLED
  Serial.print("Servo: Set angle to: ");
  Serial.println(angle);
  #endif
  
  return ERROR_SUCCESS;
}