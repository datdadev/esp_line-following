#include "sensor_driver.h"
#include <Arduino.h>
#include <stdint.h>
#include "pins.h"
#include "error_codes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===================== PIN DEFINITIONS =====================
// MUX select pins
const uint8_t muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};
const uint8_t irPins[2] = {IR0_PIN, IR1_PIN};  // Outputs from two muxes

// ===================== CALIBRATION CONSTANTS =====================
// Base values for calibration - separate for line and mid sensors
const float LINE_CALIBRATION_BASE = 112.0f;
const float MID_CALIBRATION_BASE = 123.4285f;  // Same initial value, can be adjusted separately

// Calibration coefficients for each sensor (sensor 0-6 correspond to sensors 1-7 in ref)
const float LINE_CALIBRATION_COEFFS[7] = {1.01705f, 1.02579f, 0.97291f, 0.98378f, 1.0038f, 1.00516f, 0.99195f};
const float MID_CALIBRATION_COEFFS[7] = {1.1743f, 1.4031f, 0.9846f, 1.2453f, 1.0685f, 0.9738f, 0.9743f};  // Same initial values, can be adjusted separately

// Calibration offset values for each sensor (sensor 0-6 correspond to sensors 1-7 in ref)
const int16_t LINE_CALIBRATION_OFFSETS[7] = {122, 92, 118, 99, 125, 103, 122};
const int16_t MID_CALIBRATION_OFFSETS[7] = {36, 32, 33, 30, 41, 13, 15};  // Same initial values, can be adjusted separately

// ===================== ERROR TRANSFORMATION COEFFICIENTS =====================
// Final transformation coefficients for error calculation
const float ERROR_SCALE_FACTOR = 1.0f;
const float ERROR_OFFSET = 0.0f;

// ===================== THRESHOLD CONSTANTS =====================
// Thresholds used for detection functions
const uint16_t SONAR_THRESHOLD_OBSTACLE = 450;   // mm

// ===================== SENSOR DATA VARIABLES =====================
int16_t lineSensor[7];  // Front array sensors
int16_t midSensor[7];   // Middle array sensors
float sonarDistance = 0.0;  // Global sonar distance reading

// ===================== MUX READING FUNCTIONS =====================
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

// ===================== LINE SENSORS FUNCTIONS =====================
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

// ===================== SONAR FUNCTIONS =====================
int8_t getSonarDistance(float* distance) {
    static float lastDistance = 800.0f; // For simple low-pass filter - reasonable initial distance
    static bool firstReading = true;    // Flag to handle first reading specially
    const float alpha = 0.25f;          // Filter coefficient (0 < alpha <= 1), reduced for more smoothing against motor noise
    const float maxDistance = 800.0f;  // Max distance in mm

    if (!distance) return ERROR_INVALID_PARAMETER;

    // Trigger pulse
    pinMode(ULTRASONIC_PIN, OUTPUT);
    digitalWrite(ULTRASONIC_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(ULTRASONIC_PIN, LOW);

    pinMode(ULTRASONIC_PIN, INPUT);
    uint32_t duration = pulseIn(ULTRASONIC_PIN, HIGH, 5000); // Timeout 60ms (~10 m)

    if (duration == 0) {
        // Timeout occurred
        *distance = maxDistance; // Keep last valid reading instead of hardcoded value
        return ERROR_ULTRASONIC_TIMEOUT;
    }

    // Convert to cm (this gives cm, multiply by 10 to get mm for comparison)
    float newDistance = duration * 0.17f; // 0.034 / 2 in cm

    // Clamp maximum distance
    if (newDistance > maxDistance) newDistance = maxDistance;

    // Simple low-pass filter with more aggressive smoothing to reduce motor noise
    // For the first reading, use the raw value instead of filtered to avoid starting at 0
    if (firstReading) {
        lastDistance = newDistance;
        firstReading = false;
    } else {
        lastDistance = alpha * newDistance + (1 - alpha) * lastDistance;
    }
    *distance = lastDistance;

    return ERROR_SUCCESS;
}

// Static mutex for protecting access to sonarDistance
static portMUX_TYPE sonarMux = portMUX_INITIALIZER_UNLOCKED;

void updateSonarDistance() {
  float distance = 0.0;
  int8_t result = getSonarDistance(&distance);
  
  // Use critical section to safely update the global sonarDistance
  portENTER_CRITICAL(&sonarMux);
  if (result == ERROR_SUCCESS) {
    sonarDistance = distance;
  }
  // Even if there's an error, we still update the global variable with the measured value
  // (which will be 0.0 in case of timeout), so the value is always current
  else {
    sonarDistance = distance;
  }
  portEXIT_CRITICAL(&sonarMux);
}

// Function to safely read the sonar distance from main thread
float getSonarDistanceReading() {
  float currentDistance;
  // Use critical section to safely read the global sonarDistance
  portENTER_CRITICAL(&sonarMux);
  currentDistance = sonarDistance;
  portEXIT_CRITICAL(&sonarMux);
  return currentDistance;
}

// ===================== LINE DETECTION FUNCTIONS =====================
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

// ===================== OBSTACLE DETECTION FUNCTIONS =====================
bool detectObstacle() {
  // Get the current sonar distance reading from the background task
  float currentDistance = getSonarDistanceReading();

  bool obstacleDetected = (currentDistance > 0 && currentDistance < SONAR_THRESHOLD_OBSTACLE);
  obstacleDetected ? digitalWrite(LED_RED, LOW) : digitalWrite(LED_RED, HIGH); // Turn on LED if obstacle detected (reversed)

  // Serial.println("Sonar Distance: " + String(currentDistance) + " mm, Obstacle: " + String(obstacleDetected));
  return obstacleDetected;
}

// ===================== ERROR COMPUTATION =====================
// Pre-computed coefficients for error calculation
const float ERROR_COEFFS[7] = {3.0f, 2.0f, 1.0f, 0.0f, -1.0f, -2.0f, -3.0f};
const float SCALING_FACTOR = 13.0f * ERROR_SCALE_FACTOR;

float computeError() {
  float num = 0.0f;
  float den = 0.0f;
  uint8_t D = 0;
  
  // Single pass through all sensors
  for (uint8_t i = 0; i < 7; i++) {
    float sensorValue = (float)lineSensor[i];
    num += ERROR_COEFFS[i] * sensorValue;
    den += sensorValue;
    
    if (sensorValue >= 2048.0f) D++;
  }
  
  // Handle special cases
  if (D == 0) return 999.0f;
  if (D >= 5) return 1000.0f;
  
  // Calculate final error
  return (den != 0.0f) ? ((num * SCALING_FACTOR / den) + ERROR_OFFSET) : ERROR_OFFSET;
}