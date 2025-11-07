#ifndef LYAPUNOV_CONTROLLER_H
#define LYAPUNOV_CONTROLLER_H

#include <stdint.h>
#include "error_codes.h"

// Control constants - TUNE ME
#define LYP_K2               300.0f     // lateral gain (depends on e units)
#define LYP_VREF             1.0f       // constant forward velocity (m/s)
#define LYP_OMEGA_REF        0.0f       // reference angular velocity (rad/s)
#define LYP_WHEELBASE        0.21f      // wheelbase (m)
#define LYP_STEERING_LIMIT   (PI / 6)   // ±30°, in radians

// ===================== LYAPUNOV CONTROLLER =====================
int8_t LyapunovController(float e, float* result);  // Returns error code, result via pointer

#endif // LYAPUNOV_CONTROLLER_H