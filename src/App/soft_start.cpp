#include <Arduino.h>
#include "soft_start.h"
#include "../Config/system_config.h"

// ===================== SOFT START =====================
bool soft_start_active = false;
uint32_t soft_start_start_time = 0;

int16_t getSoftStartSpeed(int16_t targetSpeed) {
  if (!soft_start_active) {
    return targetSpeed;
  }
  
  uint32_t elapsed = millis() - soft_start_start_time;
  if (elapsed >= SOFT_START_DURATION) {
    soft_start_active = false;  // Disable soft start after duration
    return targetSpeed;
  }
  
  // Calculate speed as a percentage of target speed based on elapsed time
  float progress = (float)elapsed / SOFT_START_DURATION;
  return (int16_t)(progress * targetSpeed);
}