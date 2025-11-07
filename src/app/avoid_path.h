#ifndef AVOID_PATH_H
#define AVOID_PATH_H

#include <stdint.h>
#include <math.h>

// Path function types for obstacle avoidance
enum PathFunctionType {
  SINE_WAVE = 0,
  TRIANGLE_WAVE,
  SQUARE_WAVE,
  SAWTOOTH_WAVE,
  COMBINED_WAVES,  // Combines multiple functions continuously
  SEQUENTIAL_WAVES
};

// Timestamp for avoid path functionality
extern uint32_t avoid_path_start_time;

// Path following variable
extern float sine_wave_phase;

// Function to calculate path angle based on time and path type
float calculatePathAngle(float timeElapsed, PathFunctionType pathType = SINE_WAVE);

#endif // AVOID_PATH_H