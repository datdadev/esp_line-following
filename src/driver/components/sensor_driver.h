#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <stdint.h>

// ===================== INCLUDES =====================
#include "error_codes.h"

// ===================== SENSOR CONSTANTS =====================
// Sensor array size constants
static const uint8_t SENSOR_ARRAY_SIZE = 7;

// Calibration constants for line sensors
extern const float LINE_CALIBRATION_BASE;
extern const float LINE_CALIBRATION_COEFFS[7];
extern const int16_t LINE_CALIBRATION_OFFSETS[7];

// Calibration constants for middle sensors
extern const float MID_CALIBRATION_BASE;  // Same initial value, can be adjusted separately
extern const float MID_CALIBRATION_COEFFS[7];  // Same initial values, can be adjusted separately
extern const int16_t MID_CALIBRATION_OFFSETS[7];  // Same initial values, can be adjusted separately

// Error transformation coefficients
extern const float ERROR_SCALE_FACTOR;
extern const float ERROR_OFFSET;

// Threshold constants
extern const uint16_t SONAR_THRESHOLD_OBSTACLE;   // mm

// ===================== SENSOR DATA STRUCTURES =====================
// External variables for sensor readings
extern int16_t lineSensor[7];  // Front array sensors
extern int16_t midSensor[7];   // Middle array sensors
extern float sonarDistance;

// ===================== SENSOR READING FUNCTIONS =====================
int16_t readMux(uint8_t channel, uint8_t arrayIdx);
int8_t readLineSensors();  // Returns error code
int8_t getSonarDistance(float* distance);  // Returns error code, distance via pointer
void updateSonarDistance(); // Updates the global sonarDistance variable with current reading

// ===================== OBSTACLE DETECTION FUNCTIONS =====================
bool detectObstacle();
bool detectJunction();
bool detectFinishLine();
bool detectLostLine();

// ===================== ERROR COMPUTATION FUNCTIONS =====================
float computeError();

#endif // SENSOR_DRIVER_H