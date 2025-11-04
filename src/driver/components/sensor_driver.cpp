#include "sensor_driver.h"
#include <Arduino.h>
#include <stdint.h>
#include "pins.h"

// ===================== GLOBAL VARIABLES =====================
int16_t lineSensor[7];  // Front array sensors
int16_t midSensor[7];   // Middle array sensors
float sonarDistance = 0.0;

// ===================== MUX SELECT PINS =====================
const uint8_t muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};
const uint8_t irPins[2] = {IR1_PIN, IR2_PIN};  // Outputs from two muxes

// ===================== READ MUX CHANNEL =====================
int16_t readMux(uint8_t channel, uint8_t arrayIdx) {
  for (uint8_t i = 0; i < 3; i++)
    digitalWrite(muxPins[i], (channel >> i) & 1);
  delayMicroseconds(5);
  return analogRead(irPins[arrayIdx]);
}

// ===================== READ LINE SENSORS =====================
void readLineSensors() {
  // 2 sets of 7 sensors connected via 2 multiplexers with shared select lines
  const uint8_t NUM_IR = 7;  // 7 TCRT5000 sensors per array
  
  for (uint8_t i = 0; i < NUM_IR; i++) {
    // Both muxes get the same selection signals, but we read from different output pins
    lineSensor[i] = readMux(i, 0);  // Read from front array (first mux output)
    midSensor[i] = readMux(i, 1);   // Read from middle array (second mux output)
  }
}

// ===================== READ SONAR =====================
float getSonarDistance() {
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  uint32_t duration = pulseIn(ULTRASONIC_PIN, HIGH, 30000);
  return duration * 0.034 / 2.0;
}

// ===================== DETECTION FUNCTIONS =====================
bool detectObstacle() {
  sonarDistance = getSonarDistance();
  const uint8_t SONAR_TH_OBS = 45;   // cm
  return (sonarDistance > 0 && sonarDistance < SONAR_TH_OBS);
}

bool detectJunction() {
  // Count how many sensors detect the line
  const uint8_t NUM_IR = 7;
  const uint8_t JUNCTION_COUNT = 5;
  
  uint8_t count = 0;
  bool sensorStates[NUM_IR];
  for (uint8_t i = 0; i < NUM_IR; i++) {
    sensorStates[i] = (lineSensor[i] < 500); // black threshold
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
    if (lineSensor[i] < 500) count++;
  return (count >= 6);
}

bool detectLostLine() {
  const uint8_t NUM_IR = 7;
  // If no sensors detect the line (all sensors see white/reflective surface)
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUM_IR; i++)
    if (lineSensor[i] < 500) count++; // black threshold
  return (count == 0);
}

// ===================== ERROR COMPUTATION =====================
float computeError() {
  int8_t weights[7] = {-3, -2, -1, 0, 1, 2, 3};
  const uint8_t NUM_IR = 7;
  uint8_t blackCount = 0;
  int8_t sumW = 0;

  for (uint8_t i = 0; i < NUM_IR; i++) {
    if (lineSensor[i] < 500) { // black
      sumW += weights[i];
      blackCount++;
    }
  }

  if (blackCount == 0) return 0;
  return (float)sumW / blackCount;
}