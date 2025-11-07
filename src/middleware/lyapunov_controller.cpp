#include <Arduino.h>
#include "system_config.h"
#include "lyapunov_controller.h"

// ===================== LYAPUNOV CONTROLLER =====================
// Full Lyapunov-based control for car-like robot
// e3 estimated from e2 dynamics: e3 = atan((de/dt) / v_ref)
// ω = ω_ref + K2*v_ref*e2 + K3*sin(e3)
// γ = atan(L*ω / v_ref)

static float prevE2 = 0.0f;

int8_t LyapunovController(float e2, float* result) {
  if (result == nullptr) {
    return ERROR_INVALID_PARAMETER;
  }

  if (Ts <= 0.0f || LYP_VREF <= 0.0f) {
    return ERROR_PID_INVALID_CONFIG;
  }

  // --- Compute derivative of e2 ---
  float de2 = (e2 - prevE2) / Ts;
  prevE2 = e2;

  // --- Estimate orientation error e3 ---
  float e3 = atan(de2 / LYP_VREF);

  // --- Compute angular velocity (ω) ---
  float omega = LYP_OMEGA_REF + LYP_K2 * LYP_VREF * e2 + LYP_K3 * sin(e3);

  // --- Convert to steering angle (γ) ---
  float gamma = atan((LYP_WHEELBASE * omega) / LYP_VREF);

  // --- Saturate output to steering limits ---
  gamma = constrain(gamma, -LYP_STEERING_LIMIT, LYP_STEERING_LIMIT);

  *result = gamma;
  return ERROR_SUCCESS;
}
