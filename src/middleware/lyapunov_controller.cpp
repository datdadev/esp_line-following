#include <Arduino.h>
#include "system_config.h"
#include "lyapunov_controller.h"

// ===================== LYAPUNOV CONTROLLER =====================
// Implements Lyapunov-based control algorithm for line following:
// Control law: u = K1*atan(e) + K2*e + K3*de/dt
// Where:
//   e = line error (deviation from center)
//   de/dt = derivative of error
//   K1, K2, K3 = Lyapunov controller parameters
float prevE = 0.0;
// Derivative filtering variables
float prevDerivative = 0.0;

int8_t LyapunovController(float e, float* result) {
  if (result == nullptr) {
    return ERROR_INVALID_PARAMETER;
  }
  
  // Calculate derivative of error
  float derivative = (e - prevE) / Ts;
  
  // Check for calculation errors (e.g., division by zero if Ts is 0)
  if (Ts == 0.0) {
    return ERROR_PID_INVALID_CONFIG;
  }
  
  // Apply low-pass filter to derivative to reduce noise
  float filteredDerivative = DERIVATIVE_FILTER_ALPHA * derivative + (1.0 - DERIVATIVE_FILTER_ALPHA) * prevDerivative;
  prevDerivative = filteredDerivative;
  
  // Lyapunov-based control law: w_cmd = K1 * atan(e) + K2 * e + K3 * de_f
  float lyapunov_cmd = LYP_K1 * atan(e) + LYP_K2 * e + LYP_K3 * filteredDerivative;
  
  // Store previous error for next derivative calculation
  prevE = e;
  
  // Constrain the output to acceptable servo range
  *result = constrain(lyapunov_cmd, -LYP_STEERING_LIMIT, LYP_STEERING_LIMIT);
  
  return ERROR_SUCCESS;
}