#include <Arduino.h>
#include "system_config.h"
#include "lyapunov_controller.h"

// ===================== LYAPUNOV CONTROLLER =====================
// e3 estimated from e2 dynamics: e3 = atan((de/dt) / v_ref)
// ω = ω_ref + K2*v_ref*e2 + K3*sin(e3)
// γ = atan(L*ω / v_ref)

static float prevE2Filtered = 0.0f;
static float prevDe2Filtered = 0.0f;
static float prevGammaDeg = 90.0f;  // initial servo midpoint

// Filter coefficients (tune for smoothness)
constexpr float ALPHA_E2 = 0.05f;   // filtering e2
constexpr float ALPHA_DE2 = 0.1f;   // filtering derivative de2
constexpr float MAX_STEP_DEG = 2.0f; // max servo step per loop in degrees

int8_t LyapunovController(float e2, float* result) {
    if (result == nullptr) return ERROR_INVALID_PARAMETER;
    if (Ts <= 0.0f || LYP_VREF <= 0.0f) return ERROR_PID_INVALID_CONFIG;

    // --- Low-pass filter e2 ---
    float e2Filtered = ALPHA_E2 * e2 + (1.0f - ALPHA_E2) * prevE2Filtered;
    prevE2Filtered = e2Filtered;

    // --- Compute derivative of e2 and filter it ---
    static float prevE2 = e2Filtered;
    float de2 = (e2Filtered - prevE2) / Ts;
    prevE2 = e2Filtered;

    float de2Filtered = ALPHA_DE2 * de2 + (1.0f - ALPHA_DE2) * prevDe2Filtered;
    prevDe2Filtered = de2Filtered;

    // --- Estimate orientation error e3 ---
    float e3 = atan(de2Filtered / LYP_VREF);

    // --- Compute angular velocity (ω) ---
    float omega = LYP_OMEGA_REF + LYP_K2 * LYP_VREF * e2Filtered + LYP_K3 * sin(e3);

    // --- Compute steering angle (γ) in radians ---
    float gamma = atan((LYP_WHEELBASE * omega) / LYP_VREF);

    // --- Saturate output to controller limits ---
    gamma = constrain(gamma, -LYP_STEERING_LIMIT, LYP_STEERING_LIMIT);

    // Convert to degrees and apply rate limiting
    float gammaDeg = degrees(gamma);
    // Rate limiter: limit how fast servo can move
    if (gammaDeg > prevGammaDeg + MAX_STEP_DEG) gammaDeg = prevGammaDeg + MAX_STEP_DEG;
    if (gammaDeg < prevGammaDeg - MAX_STEP_DEG) gammaDeg = prevGammaDeg - MAX_STEP_DEG;
    prevGammaDeg = gammaDeg;

    // Update output (invert if needed to match servo direction)
    *result = -gammaDeg;

    return ERROR_SUCCESS;
}
