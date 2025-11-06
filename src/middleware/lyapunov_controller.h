#ifndef LYAPUNOV_CONTROLLER_H
#define LYAPUNOV_CONTROLLER_H

#include <stdint.h>
#include "error_codes.h"

// ===================== LYAPUNOV CONTROLLER =====================
int8_t LyapunovController(float e, float* result);  // Returns error code, result via pointer

#endif // LYAPUNOV_CONTROLLER_H