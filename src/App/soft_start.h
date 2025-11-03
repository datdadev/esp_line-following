#ifndef SOFT_START_H
#define SOFT_START_H

#include <stdint.h>

// ===================== SOFT START =====================
int16_t getSoftStartSpeed(int16_t targetSpeed);
extern bool soft_start_active;
extern uint32_t soft_start_start_time;

#endif // SOFT_START_H