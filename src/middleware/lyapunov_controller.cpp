#include <Arduino.h>
#include "system_config.h"
#include "lyapunov_controller.h"

// ===================== LYAPUNOV CONTROLLER =====================
static float prevE2Filtered = 0.0f;
static float prevDe2Filtered = 0.0f;
static float prevGammaDeg = 0.0f;

// Pre-computed constants for performance
constexpr float ALPHA_E2 = 0.05f;
constexpr float ALPHA_DE2 = 0.1f;
constexpr float INV_TS = 1.0f / Ts;  // Precompute division
constexpr float MAX_STEP_DEG = 2.0f;

// Pre-computed controller constants
constexpr float K2_VREF = LYP_K2 * LYP_VREF;      // LYP_K2 * LYP_VREF
constexpr float L_VREF_RATIO = LYP_WHEELBASE / LYP_VREF;  // L * ω / v_ref

// Small-angle approximation threshold (about 10 degrees)
constexpr float SMALL_ANGLE_THRESHOLD = 0.174533f; // ~10 degrees in radians


// AGRESSIVE VERSION
int8_t LyapunovController(float e2, float* result) {
    if (result == nullptr) return ERROR_INVALID_PARAMETER;
    
    // Ultra-fast filtering (single operation)
    float e2Filtered = prevE2Filtered + ALPHA_E2 * (e2 - prevE2Filtered);
    prevE2Filtered = e2Filtered;
    
    // Derivative with precomputed inverse
    static float prevE2 = e2Filtered;
    float de2Filtered = prevDe2Filtered + ALPHA_DE2 * ((e2Filtered - prevE2) * INV_TS - prevDe2Filtered);
    prevE2 = e2Filtered;
    prevDe2Filtered = de2Filtered;
    
    // Small-angle approximation for everything (much faster)
    float e3 = de2Filtered / LYP_VREF;
    float sin_e3 = e3;  // Approximation
    
    float omega = LYP_OMEGA_REF + K2_VREF * e2Filtered + LYP_K3 * sin_e3;
    float gamma = L_VREF_RATIO * omega;  // Approximation
    
    // Saturation
    gamma = (gamma < -LYP_STEERING_LIMIT) ? -LYP_STEERING_LIMIT : 
            (gamma > LYP_STEERING_LIMIT) ? LYP_STEERING_LIMIT : gamma;
    
    // Rate limiting
    float gammaDeg = gamma * RAD_TO_DEG;
    float delta = gammaDeg - prevGammaDeg;
    gammaDeg = (delta > MAX_STEP_DEG) ? prevGammaDeg + MAX_STEP_DEG :
               (delta < -MAX_STEP_DEG) ? prevGammaDeg - MAX_STEP_DEG : gammaDeg;
    prevGammaDeg = gammaDeg;
    
    *result = -gammaDeg;
    return ERROR_SUCCESS;
}

// // SOFT VERSION
// int8_t LyapunovController(float e2, float* result) {
//     if (result == nullptr) return ERROR_INVALID_PARAMETER;
//     if (Ts <= 0.0f || LYP_VREF <= 0.0f) return ERROR_PID_INVALID_CONFIG;

//     // --- Fast low-pass filter e2 (reduced operations) ---
//     float e2Filtered = prevE2Filtered + ALPHA_E2 * (e2 - prevE2Filtered);
//     prevE2Filtered = e2Filtered;

//     // --- Compute derivative using precomputed inverse ---
//     static float prevE2 = e2Filtered;
//     float de2 = (e2Filtered - prevE2) * INV_TS;
//     prevE2 = e2Filtered;

//     // --- Fast derivative filtering ---
//     float de2Filtered = prevDe2Filtered + ALPHA_DE2 * (de2 - prevDe2Filtered);
//     prevDe2Filtered = de2Filtered;

//     // --- Fast orientation error estimation ---
//     float ratio = de2Filtered / LYP_VREF;
//     float e3;
    
//     // Use small-angle approximation for faster computation when possible
//     if (fabs(ratio) < SMALL_ANGLE_THRESHOLD) {
//         e3 = ratio;  // atan(x) ≈ x for small x
//     } else {
//         e3 = atan(ratio);
//     }

//     // --- Compute angular velocity with precomputed constants ---
//     float sin_e3;
//     if (fabs(e3) < SMALL_ANGLE_THRESHOLD) {
//         sin_e3 = e3;  // sin(x) ≈ x for small x
//     } else {
//         sin_e3 = sin(e3);
//     }
    
//     float omega = LYP_OMEGA_REF + K2_VREF * e2Filtered + LYP_K3 * sin_e3;

//     // --- Fast steering angle computation ---
//     float omega_ratio = L_VREF_RATIO * omega;
//     float gamma;
    
//     if (fabs(omega_ratio) < SMALL_ANGLE_THRESHOLD) {
//         gamma = omega_ratio;  // atan(x) ≈ x for small x
//     } else {
//         gamma = atan(omega_ratio);
//     }

//     // --- Fast saturation ---
//     // Using conditional moves instead of function call
//     gamma = (gamma < -LYP_STEERING_LIMIT) ? -LYP_STEERING_LIMIT : 
//             (gamma > LYP_STEERING_LIMIT) ? LYP_STEERING_LIMIT : gamma;

//     // --- Convert to degrees and rate limiting ---
//     float gammaDeg = gamma * RAD_TO_DEG;  // degrees() is usually a macro
    
//     // Fast rate limiting without function calls
//     float delta = gammaDeg - prevGammaDeg;
//     if (delta > MAX_STEP_DEG) gammaDeg = prevGammaDeg + MAX_STEP_DEG;
//     else if (delta < -MAX_STEP_DEG) gammaDeg = prevGammaDeg - MAX_STEP_DEG;
    
//     prevGammaDeg = gammaDeg;

//     // Update output
//     *result = -gammaDeg;

//     return ERROR_SUCCESS;
// }
