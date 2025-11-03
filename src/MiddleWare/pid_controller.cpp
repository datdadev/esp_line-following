#include <Arduino.h>
#include "pid_controller.h"
#include "../Config/system_config.h"

// ===================== PID CONTROLLER =====================
float prevE = 0.0, integral = 0.0;
// Derivative filtering variables
float prevDerivative = 0.0;

float PID(float e) {
  float derivative = (e - prevE) / Ts;
  
  // Apply low-pass filter to derivative to reduce noise
  float filteredDerivative = DERIVATIVE_FILTER_ALPHA * derivative + (1.0 - DERIVATIVE_FILTER_ALPHA) * prevDerivative;
  prevDerivative = filteredDerivative;
  
  integral += e * Ts;
  float u = Kp * e + Ki * integral + Kd * filteredDerivative;
  prevE = e;
  return constrain(u, -30, 30); // servo angle offset
}