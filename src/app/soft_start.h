#ifndef SOFT_START_H
#define SOFT_START_H

#include <stdint.h>
#include "error_codes.h"

// ===================== SOFT START =====================
int8_t getSoftStartSpeed(float targetSpeed, float* result);  // Returns error code, result via pointer
extern bool soft_start_active;
extern uint32_t soft_start_start_time;

#endif // SOFT_START_H