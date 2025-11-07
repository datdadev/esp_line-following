#include "avoid_path.h"

// Timestamp for avoid path functionality
uint32_t avoid_path_start_time = 0;

// Path following variable
float sine_wave_phase = 0.0;

// Function to calculate path angle based on time and path type
float calculatePathAngle(float timeElapsed, PathFunctionType pathType) {
  float frequency = 0.5;  // 0.5 Hz - complete one oscillation every 2 seconds
  float amplitude = 30.0; // 30 degrees amplitude from center
  
  switch(pathType) {
    case SINE_WAVE:
      return 90 + amplitude * sin(2 * M_PI * frequency * timeElapsed);
      
    case TRIANGLE_WAVE: {
      // Triangle wave: creates a linear oscillation between min and max
      float period = 1.0 / frequency;
      float normalizedTime = fmod(timeElapsed, period);
      float value;
      if (normalizedTime < period / 2.0) {
        value = 4 * amplitude * normalizedTime / period - amplitude;  // Rising edge
      } else {
        value = -4 * amplitude * (normalizedTime - period / 2.0) / period + amplitude;  // Falling edge
      }
      return 90 + value;
    }
    
    case SQUARE_WAVE: {
      // Square wave: alternates between max and min values
      float period = 1.0 / frequency;
      float normalizedTime = fmod(timeElapsed, period);
      if (normalizedTime < period / 2.0) {
        return 90 + amplitude;  // Positive half
      } else {
        return 90 - amplitude;  // Negative half
      }
    }
    
    case SAWTOOTH_WAVE: {
      // Sawtooth wave: ramps up then drops back to start
      float period = 1.0 / frequency;
      float normalizedTime = fmod(timeElapsed, period);
      return 90 + amplitude * (2 * normalizedTime / period - 1);
    }
    
    case COMBINED_WAVES: {
      // Combination of multiple functions: sine + triangle
      // This creates a continuously changing pattern
      float sineComponent = amplitude * 0.6 * sin(2 * M_PI * frequency * timeElapsed);
      float triangleComponent = amplitude * 0.4 * sin(2 * M_PI * frequency * 1.5 * timeElapsed + M_PI/3);
      return 90 + sineComponent + triangleComponent;
    }

    case SEQUENTIAL_WAVES: {
      float val;
      
      // Thời gian một pha (ví dụ 5 giây mỗi sóng)
      float period = 5.0;  
      int phase = int(timeElapsed / period) % 2;  // 0 = sin, 1 = square

      if (phase == 0) {
        // --- Sóng sin ---
        val = amplitude * sin(2 * M_PI * frequency * timeElapsed);
      } else {
        // --- Sóng vuông ---
        float s = sin(2 * M_PI * frequency * timeElapsed);
        val = (s >= 0) ? amplitude : -amplitude;  // chuyển sin thành vuông
      }

      return 90 + val;  // servo trung tâm ở 90 độ
    }

    
    default:
      return 90 + amplitude * sin(2 * M_PI * frequency * timeElapsed);  // Default to sine
  }
}