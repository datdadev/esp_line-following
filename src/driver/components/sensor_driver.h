#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <stdint.h>

// ===================== INCLUDES =====================
#include "error_codes.h"

// ===================== SENSOR READINGS =====================
int16_t readMux(uint8_t channel, uint8_t arrayIdx);
int8_t readLineSensors();  // Returns error code
int8_t getSonarDistance(float* distance);  // Returns error code, distance via pointer
void updateSonarDistance(); // Updates the global sonarDistance variable with current reading

// ===================== LINE SENSORS =====================
extern int16_t lineSensor[7];  // Front array sensors
extern int16_t midSensor[7];   // Middle array sensors
extern float sonarDistance;

// ===================== DETECTION FUNCTIONS =====================
bool detectObstacle();
bool detectJunction();
bool detectFinishLine();
bool detectLostLine();

// ===================== ERROR COMPUTATION =====================
float computeError();

#endif // SENSOR_DRIVER_H