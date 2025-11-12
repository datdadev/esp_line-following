#include <Arduino.h>
#include "system_config.h"
#include "soft_start.h"

// ===================== SOFT START =====================
bool soft_start_active = false;
uint32_t soft_start_start_time = 0;

int8_t getSoftStartSpeed(float targetSpeed, float* result) {
  if (result == nullptr) {
    return ERROR_INVALID_PARAMETER;
  }
  
  if (!soft_start_active) {
    *result = targetSpeed;
    return ERROR_SUCCESS;
  }
  
  uint32_t elapsed = millis() - soft_start_start_time;
  if (elapsed >= SOFT_START_DURATION) {
    soft_start_active = false;  // Disable soft start after duration
    *result = targetSpeed;
    return ERROR_SUCCESS;
  }
  
  // Calculate speed as a percentage of target speed based on elapsed time
  float progress = (float)elapsed / SOFT_START_DURATION;
  if (SOFT_START_DURATION == 0) {
    // Avoid division by zero
    return ERROR_INVALID_PARAMETER;
  }
  
  *result = progress * targetSpeed;
  return ERROR_SUCCESS;
}