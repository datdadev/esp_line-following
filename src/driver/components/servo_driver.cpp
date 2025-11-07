#include "servo_driver.h"
#include "pins.h"
#include "error_codes.h"
#include <Arduino.h>
#include "error_codes.h"

// ===================== SERVO CONTROL =====================
Servo steering;
static int8_t currentServoAngle = 90;  // Track current servo angle, default to center

int8_t initServo() {
  if (steering.attached()) {
  
    return ERROR_ALREADY_INITIALIZED;  // Servo already initialized
  }
  
  // Attach servo with min and max pulse widths for wide range servo (500-2400µs)
  // This allows for extended 0-180 degree range
  if (steering.attach(SERVO_PIN, 500, 2400) < 0) {
  
    return ERROR_SERVO_INIT_FAILED;  // Failed to attach servo
  }
  
  steering.write(90);  // Set initial position to center (90 degrees)

  

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
  
    return ERROR_SERVO_OUT_OF_RANGE;  // Angle out of acceptable range
  }
  
  currentServoAngle = angle;
  steering.write(angle);  // Set servo to requested angle

  
  return ERROR_SUCCESS;
}

int8_t getCurrentServoAngle(int8_t* angle) {
  if (angle == nullptr) {
    return ERROR_INVALID_PARAMETER;
  }
  
  *angle = currentServoAngle;
  return ERROR_SUCCESS;
}