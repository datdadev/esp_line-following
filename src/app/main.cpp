#include <Arduino.h>
#include <ESP32Servo.h>
#include <stdint.h>
#include "system_config.h"
#include "pins.h"
#include "motor_driver.h"
#include "sensor_driver.h"
#include "servo_driver.h"
#include "lyapunov_controller.h"  // Lyapunov-based line following controller
#include "soft_start.h"
#include "network_handler.h"
#include "error_codes.h"
#include "robot_state.h"

// ===================== GLOBAL VARIABLES =====================
State state = INIT;

bool afterJunction = false;
bool nearGoalFlag = false;
uint32_t lastSample = 0;

// Timestamps for non-blocking delays
uint32_t avoid_prepare_start = 0;
uint32_t turn_left_prepare_start = 0;

// Soft-start variables
bool first_run_after_init = true;  // To only apply soft-start on first run

// Lyapunov controller debugging variables
float lyapunov_last_error = 0.0;

// ===================== QUADRATURE ENCODER VARIABLES =====================
volatile int8_t lastEncoded = 0;
volatile int32_t encoderCount = 0;
volatile int16_t lastMSB = 0;
volatile int16_t lastLSB = 0;

// ===================== ENCODER UTILITIES =====================
void resetEncoder() {
  noInterrupts();  // Disable interrupts for atomic operation
  encoderCount = 0;
  lastEncoded = (digitalRead(ENCA) << 1) | digitalRead(ENCB);  // Initialize with current state
  interrupts();   // Re-enable interrupts
}

// ===================== SPEED CONTROL VARIABLES =====================
volatile int32_t lastEncoderCount = 0;
float currentSpeed = 0.0;  // Current speed in m/s
uint32_t lastSpeedCalcTime = 0;

// ===================== SPEED CONTROL FUNCTIONS =====================
float calculateSpeed() {
  // Calculate speed based on encoder counts over time
  uint32_t currentTime = millis();
  float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0; // Convert to seconds
  
  if (deltaTime > 0) {
    noInterrupts();  // Disable interrupts for atomic read
    int32_t currentCount = encoderCount;
    interrupts();    // Re-enable interrupts
    
    int32_t deltaCount = currentCount - lastEncoderCount;
    
    // Calculate distance traveled
    float wheelRev = deltaCount / COUNTS_PER_REV;  // Wheel revolutions
    float distance = wheelRev * (WHEEL_DIAMETER * M_PI);  // Distance in meters
    
    // Calculate speed in m/s
    currentSpeed = distance / deltaTime;
    
    // Update for next calculation
    lastEncoderCount = currentCount;
    lastSpeedCalcTime = currentTime;
  }
  
  return currentSpeed;
}

void resetSpeedCalculation() {
  noInterrupts();  // Disable interrupts for atomic operation
  lastEncoderCount = encoderCount;
  interrupts();    // Re-enable interrupts
  lastSpeedCalcTime = millis();
  currentSpeed = 0.0;
}

// ===================== NEAR GOAL =====================
bool nearGoal() {
  // Calculate distance traveled based on encoder counts
  float wheelRev = encoderCount / COUNTS_PER_REV;  // Wheel revolutions
  float distanceTraveled = wheelRev * (WHEEL_DIAMETER * M_PI);  // Distance in meters
  
  return (afterJunction && distanceTraveled >= DIST_NEAR_GOAL / 100.0); // Convert cm to meters
}

// ===================== ENCODER ISR =====================
void IRAM_ATTR encoderISR() {
  // Read both encoder pins
  int16_t MSB = digitalRead(ENCA);  // Most significant bit
  int16_t LSB = digitalRead(ENCB);  // Least significant bit
  
  // Combine the two pin states into a single value
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = (lastEncoded << 2) | encoded;
  
  // Increment or decrement based on the transition pattern
  // Clockwise/Forward: 00->01->11->10->00 (0->1->3->2->0)
  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
    encoderCount++;  // Forward direction
  }
  // Counter-clockwise/Reverse: 00->10->11->01->00 (0->2->3->1->0)
  else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) {
    encoderCount--;  // Reverse direction
  }
  
  // Store the current state for next interrupt
  lastEncoded = encoded;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("===== ESP32 Robot Booting =====");

  // Setup pins
  Serial.println("[1/6] Configuring pins...");
  const uint8_t muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};
  for (uint8_t i = 0; i < 3; i++) pinMode(muxPins[i], OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR, CHANGE);
  Serial.println("  -> Pin configuration OK");

  // Servo
  Serial.println("[2/6] Initializing servo...");
  int err = initServo();
  if (err != ERROR_SUCCESS) {
    Serial.printf("  [ERR] Servo init failed! Code: %d\n", err);
    while (1);
  } else {
    Serial.println("  -> Servo init success");
  }

  err = setServoAngle(90);
  if (err != ERROR_SUCCESS) {
    Serial.printf("  [WARN] Servo angle set failed! Code: %d\n", err);
  } else {
    Serial.println("  -> Servo centered at 90°");
  }

  // Speed calculation
  Serial.println("[3/6] Resetting speed calculation...");
  resetSpeedCalculation();
  Serial.println("  -> Speed calculation ready");

  // Network setup
  Serial.println("[4/6] Initializing network...");
  err = initNetwork();
  if (err == ERROR_SUCCESS) {
    setupWebSocket();
    server.begin();
    Serial.println("  -> Network initialized successfully");
  } else {
    Serial.printf("  [ERR] Network init failed! Code: %d\n", err);
  }

  // State init
  Serial.println("[5/6] Setting initial state...");
  state = INIT;
  Serial.println("  -> State set to INIT");

  // Final message
  Serial.println("[6/6] System initialized successfully!");
  Serial.println("===================================");
}

// ===================== MAIN LOOP =====================
void loop() {
  if (millis() - lastSample < Ts * 1000) return;
  lastSample = millis();

  if (readLineSensors() != ERROR_SUCCESS) {
    // Handle sensor reading error
    setMotor(0);  // Stop the robot
    setServoAngle(90);  // Center the servo
    // Could implement error state or retry logic here
    // For now, stay in current state to prevent erratic behavior
    return;
  }

  // Calculate current speed based on encoder feedback
  calculateSpeed();

  switch (state) {

    case INIT:
      setMotor(0);  // Stop the motor (error checking not required for normal operation)
      setServoAngle(90);  // Center the servo (error checking not required for normal operation)
      if (digitalRead(BTN_BOOT) == LOW) {
        state = LINE_FOLLOW;
        first_run_after_init = true;  // Enable soft start for the first run
        // Start line following
      }
      break;

    case LINE_FOLLOW: {
      // Lyapunov Controller Execution:
      // 1. Compute error from line position (negative = left of center, positive = right of center)
      float e = computeError();
      
      // 2. Apply Lyapunov control law: u = K1*atan(e) + K2*e + K3*de/dt
      float u = 0.0;
      if (LyapunovController(e, &u) != ERROR_SUCCESS) {
        // Handle Lyapunov controller error - perhaps use a default action
        u = 0.0;  // Default to no correction
      }
      
      // 3. Apply steering correction: 90° is center position, u is the offset from Lyapunov algorithm
      setServoAngle(90 + (int8_t)u); // Set servo angle based on Lyapunov output
      
      // Use soft start when first entering LINE_FOLLOW state
      if (first_run_after_init) {
        if (!soft_start_active) {
          soft_start_active = true;
          soft_start_start_time = millis();
          first_run_after_init = false;  // Only do soft-start once
        }
        int16_t softStartSpeed = 0;
        if (getSoftStartSpeed(PWM_NORMAL, &softStartSpeed) == ERROR_SUCCESS) {
          setMotor(softStartSpeed); // Attempt to set motor with soft start speed
        } else {
          setMotor(PWM_NORMAL); // Fall back to normal speed if soft start fails
        }
      } else {
        setMotor(PWM_NORMAL); // Set motor to normal speed (1 m/s)
      }

      if (detectLostLine()) {
        state = LOST_LINE;
      } else if (detectObstacle()) {
        state = AVOID_PREPARE;
      } else if (detectJunction() && !afterJunction) {
        state = TURN_LEFT_PREPARE;
      } else if (afterJunction && nearGoal()) {
        state = SLOW_DOWN;
      }
      break;
    }

    case AVOID_PREPARE:
      if (avoid_prepare_start == 0) {
        setMotor(PWM_SLOW); // Attempt to set motor speed
        setServoAngle(130); // steer right
        avoid_prepare_start = millis();
      }
      if (millis() - avoid_prepare_start >= 500) {
        avoid_prepare_start = 0; // reset timer
        state = AVOID_PATH;
      }
      break;

    case AVOID_PATH:
      setMotor(PWM_NORMAL); // Set motor to normal speed
      setServoAngle(130); // Steer right
      if (!detectObstacle()) {
        state = MERGE_SEARCH;
      }
      break;

    case MERGE_SEARCH:
      setServoAngle(70); // steer back left to find line
      if (detectJunction() || computeError() != 0) {
        state = LINE_FOLLOW;
        first_run_after_init = false;  // Don't use soft start when resuming from merge
      }
      break;

    case TURN_LEFT_PREPARE:
      if (turn_left_prepare_start == 0) {
        setMotor(PWM_SLOW); // Set motor to slow speed
        setServoAngle(50); // left turn
        turn_left_prepare_start = millis();
      }
      if (millis() - turn_left_prepare_start >= 800) {
        turn_left_prepare_start = 0; // reset timer
        afterJunction = true;
        resetEncoder();          // Reset encoder counter
        resetSpeedCalculation(); // Reset speed calculation
        setServoAngle(90); // Center servo after turn
        state = LINE_FOLLOW;
        first_run_after_init = false;  // Don't use soft start when resuming from turn
      }
      break;

    case SLOW_DOWN:
      setMotor(PWM_SLOW); // Set motor to slow speed
      setServoAngle(90); // Keep servo centered
      if (detectFinishLine()) {
        state = STOP;
      }
      break;

    case STOP:
      setMotor(0); // Stop the motor
      setServoAngle(90); // Center the servo
      while (1);
      break;

    case LOST_LINE:
      setMotor(0); // Stop the motor
      setServoAngle(90); // Center the steering
      // Optionally, you can add a small forward movement or steering to try to find the line
      // For now, we'll just stop and wait for manual intervention
      break;
  }
  
  // Send telemetry periodically (every 100ms)
  static uint32_t lastTelemetry = 0;
  if (millis() - lastTelemetry > 100) {
    sendTelemetry();
    lastTelemetry = millis();
  }
}