#include <Arduino.h>
#include "system_config.h"
#include "pid_controller.h"

// ===================== PID CONTROLLER =====================
float prevE = 0.0, integral = 0.0;
// Derivative filtering variables
float prevDerivative = 0.0;

int8_t PID(float e, float* result) {
  if (result == nullptr) {
    return ERROR_INVALID_PARAMETER;
  }
  
  float derivative = (e - prevE) / Ts;
  
  // Check for calculation errors (e.g., division by zero if Ts is 0)
  if (Ts == 0.0) {
    return ERROR_PID_INVALID_CONFIG;
  }
  
  // Apply low-pass filter to derivative to reduce noise
  float filteredDerivative = DERIVATIVE_FILTER_ALPHA * derivative + (1.0 - DERIVATIVE_FILTER_ALPHA) * prevDerivative;
  prevDerivative = filteredDerivative;
  
  integral += e * Ts;
  float u = Kp * e + Ki * integral + Kd * filteredDerivative;
  prevE = e;
  
  *result = constrain(u, -30, 30); // servo angle offset
  
  return ERROR_SUCCESS;
}