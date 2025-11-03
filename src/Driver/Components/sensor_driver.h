#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <stdint.h>

// ===================== SENSOR READINGS =====================
int16_t readMux(uint8_t channel, uint8_t arrayIdx);
void readLineSensors();
float getSonarDistance();

// ===================== LINE SENSORS =====================
extern int16_t lineSensor[7];
extern float sonarDistance;

// ===================== DETECTION FUNCTIONS =====================
bool detectObstacle();
bool detectJunction();
bool detectFinishLine();
bool detectLostLine();

// ===================== ERROR COMPUTATION =====================
float computeError();

#endif // SENSOR_DRIVER_H