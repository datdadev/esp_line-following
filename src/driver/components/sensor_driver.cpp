#include "sensor_driver.h"
#include <Arduino.h>
#include <stdint.h>
#include "pins.h"
#include "error_codes.h"

// ===================== LINEAR REGRESSION CALIBRATION COEFFICIENTS =====================
// Base values for calibration - separate for line and mid sensors
const float LINE_CALIBRATION_BASE = 123.4285f;
const float MID_CALIBRATION_BASE = 123.4285f;  // Same initial value, can be adjusted separately

// Calibration coefficients for each sensor (sensor 0-6 correspond to sensors 1-7 in ref)
const float LINE_CALIBRATION_COEFFS[7] = {1.1743f, 1.4031f, 0.9846f, 1.2453f, 1.0685f, 0.9738f, 0.9743f};
const float MID_CALIBRATION_COEFFS[7] = {1.1743f, 1.4031f, 0.9846f, 1.2453f, 1.0685f, 0.9738f, 0.9743f};  // Same initial values, can be adjusted separately

// Calibration offset values for each sensor (sensor 0-6 correspond to sensors 1-7 in ref)
const int16_t LINE_CALIBRATION_OFFSETS[7] = {36, 32, 33, 30, 41, 13, 15};
const int16_t MID_CALIBRATION_OFFSETS[7] = {36, 32, 33, 30, 41, 13, 15};  // Same initial values, can be adjusted separately

// ===================== ERROR TRANSFORMATION COEFFICIENTS =====================
// Final transformation coefficients for error calculation
const float ERROR_SCALE_FACTOR = 0.9822f;
const float ERROR_OFFSET = -1.6627f;

// ===================== GLOBAL VARIABLES =====================
int16_t lineSensor[7];  // Front array sensors
int16_t midSensor[7];   // Middle array sensors
float sonarDistance = 0.0;

// ===================== MUX SELECT PINS =====================
const uint8_t muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};
const uint8_t irPins[2] = {IR1_PIN, IR2_PIN};  // Outputs from two muxes

// ===================== READ MUX CHANNEL =====================
int16_t readMux(uint8_t channel, uint8_t arrayIdx) {
  // Validate parameters
  if (channel >= 7 || arrayIdx > 1) {
    return ERROR_INVALID_PARAMETER;  // Return error code instead of invalid value
  }
  
  for (uint8_t i = 0; i < 3; i++)
    digitalWrite(muxPins[i], (channel >> i) & 1);
  delayMicroseconds(5);
  return analogRead(irPins[arrayIdx]);
}

// ===================== READ LINE SENSORS =====================
int8_t readLineSensors() {
  // 2 sets of 7 sensors connected via 2 multiplexers with shared select lines
  const uint8_t NUM_IR = 7;  // 7 TCRT5000 sensors per array
  
  // Read raw sensor values
  int16_t rawLineSensor[7];
  int16_t rawMidSensor[7];
  
  for (uint8_t i = 0; i < NUM_IR; i++) {
    // Both muxes get the same selection signals, but we read from different output pins
    int16_t lineValue = readMux(i, 0);  // Read from front array (first mux output)
    if (lineValue < 0) {  // Check for error code returned by readMux
      return ERROR_SENSOR_READ_FAILED;
    }
    rawLineSensor[i] = lineValue;
    
    int16_t midValue = readMux(i, 1);   // Read from middle array (second mux output)
    if (midValue < 0) {  // Check for error code returned by readMux
      return ERROR_SENSOR_READ_FAILED;
    }
    rawMidSensor[i] = midValue;
  }
  
  // Apply linear regression calibration to front sensors using line constants
  for (uint8_t i = 0; i < 7; i++) {
    lineSensor[i] = (int16_t)(LINE_CALIBRATION_BASE + LINE_CALIBRATION_COEFFS[i] * (rawLineSensor[i] - LINE_CALIBRATION_OFFSETS[i]));
    midSensor[i] = (int16_t)(MID_CALIBRATION_BASE + MID_CALIBRATION_COEFFS[i] * (rawMidSensor[i] - MID_CALIBRATION_OFFSETS[i]));
  }
  
  return ERROR_SUCCESS;
}

// ===================== READ SONAR =====================
int8_t getSonarDistance(float* distance) {
  if (distance == nullptr) {
    return ERROR_INVALID_PARAMETER;
  }
  
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  // Increase timeout to 60000 microseconds (about 10 meters max range)
  uint32_t duration = pulseIn(ULTRASONIC_PIN, HIGH, 60000);
  
  if (duration == 0) {
    // Timeout occurred
    *distance = 0.0;
    return ERROR_ULTRASONIC_TIMEOUT;
  }
  
  *distance = (float)duration * 0.17; // 0.034 / 2.0; // Calculate distance in cm
  return ERROR_SUCCESS;
}

// ===================== SONAR UPDATE FUNCTION =====================
void updateSonarDistance() {
  float distance = 0.0;
  int8_t result = getSonarDistance(&distance);
  if (result == ERROR_SUCCESS) {
    sonarDistance = distance;
  }
  // Even if there's an error, we still update the global variable with the measured value
  // (which will be 0.0 in case of timeout), so the value is always current
  else {
    sonarDistance = distance;
  }
}

// ===================== DETECTION FUNCTIONS =====================
bool detectObstacle() {
  // Update global sonarDistance variable with current reading
  updateSonarDistance();
  
  const uint16_t SONAR_TH_OBS = 450;   // mm
  return (sonarDistance > 0 && sonarDistance < SONAR_TH_OBS);
}

bool detectJunction() {
  // Count how many sensors detect the line
  const uint8_t NUM_IR = 7;
  const uint8_t JUNCTION_COUNT = 5;
  
  uint8_t count = 0;
  bool sensorStates[NUM_IR];
  for (uint8_t i = 0; i < NUM_IR; i++) {
    sensorStates[i] = (lineSensor[i] < 200); // black threshold - adjusted for calibrated values
    if (sensorStates[i]) count++;
  }
  
  // Check if we have enough sensors detecting the line
  if (count < JUNCTION_COUNT) return false;
  
  // Additional pattern check: verify that the black line spans across center sensors
  // This helps distinguish junctions from thick lines or other patterns
  bool centerSpan = (sensorStates[2] || sensorStates[3] || sensorStates[4]); // Check sensors around center
  bool edgeDetected = (sensorStates[0] || sensorStates[1] || sensorStates[5] || sensorStates[6]); // Check edges too
  
  return centerSpan && edgeDetected;
}

bool detectFinishLine() {
  const uint8_t NUM_IR = 7;
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUM_IR; i++)
    if (lineSensor[i] < 200) count++; // threshold adjusted for calibrated values
  return (count >= 6);
}

bool detectLostLine() {
  const uint8_t NUM_IR = 7;
  // If no sensors detect the line (all sensors see white/reflective surface)
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUM_IR; i++)
    if (lineSensor[i] < 200) count++; // black threshold - adjusted for calibrated values
  return (count == 0);
}

// ===================== ERROR COMPUTATION =====================
float computeError() {
  // Use the already calibrated lineSensor values for error computation
  // Calculate the weighted sum for error computation using calibrated values
  float num = 3 * (lineSensor[0] - lineSensor[6]) +  // sensorValue1 - sensorValue7
              2 * (lineSensor[1] - lineSensor[5]) +  // sensorValue2 - sensorValue6
                  (lineSensor[2] - lineSensor[4]);   // sensorValue3 - sensorValue5

  float den = lineSensor[0] + lineSensor[1] + lineSensor[2] + 
              lineSensor[3] + lineSensor[4] + lineSensor[5] + lineSensor[6];

  // Calculate the base error value using the calibrated formula
  float TB = (den != 0.0f) ? (num * 13.0f / den) : 0.0f;

  // Count how many sensors detect the line (threshold is around 200 for calibrated values)
  uint8_t D = 0;
  for (uint8_t i = 0; i < 7; i++) {
    if (lineSensor[i] < 200) D++;  // calibrated threshold for "black" detection
  }

  if (D == 0) return 999;    // lost line
  else if (D >= 5) return 1000; // intersection (5 or more sensors detecting line)
  
  // Apply final transformation to get error value using constants
  return TB * ERROR_SCALE_FACTOR + ERROR_OFFSET;
}