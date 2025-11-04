#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include "error_codes.h"

// ===================== PID CONTROLLER =====================
int8_t PID(float e, float* result);  // Returns error code, result via pointer

#endif // PID_CONTROLLER_H