#include <Arduino.h>
#include <ESP32Servo.h>
#include <stdint.h>
#include "system_config.h"
#include "pins.h"
#include "motor_driver.h"
#include "sensor_driver.h"
#include "servo_driver.h"
#include "pid_controller.h"
#include "soft_start.h"
#include "error_codes.h"

// ===================== GLOBAL VARIABLES =====================
enum State {
  INIT, IDLE, LINE_FOLLOW,
  AVOID_PREPARE, AVOID_PATH, MERGE_SEARCH,
  TURN_LEFT_PREPARE, SLOW_DOWN, STOP,
  LOST_LINE
};
State state = INIT;

int32_t encoderCount = 0;
bool afterJunction = false;
bool nearGoalFlag = false;
uint32_t lastSample = 0;

// Timestamps for non-blocking delays
uint32_t avoid_prepare_start = 0;
uint32_t turn_left_prepare_start = 0;

// Soft-start variables
bool first_run_after_init = true;  // To only apply soft-start on first run

// ===================== NEAR GOAL =====================
bool nearGoal() {
  return (afterJunction && encoderCount * 0.05 >= DIST_NEAR_GOAL);
}

// ===================== ENCODER ISR =====================
void IRAM_ATTR encoderISR() {
  encoderCount++;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // Setup pins
  const uint8_t muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};
  for (uint8_t i = 0; i < 3; i++) pinMode(muxPins[i], OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, RISING);

  if (initServo() != ERROR_SUCCESS) {
    // Could enter error state or halt execution
    while(1); // Halt execution if critical component fails
  }
  if (setServoAngle(90) != ERROR_SUCCESS) {
    // Handle servo error - possibly ignore or add specific handling
  }

  state = INIT;
  // System initialized
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
      float e = computeError();
      float u = 0.0;
      if (PID(e, &u) != ERROR_SUCCESS) {
        // Handle PID error - perhaps use a default action
        u = 0.0;  // Default to no correction
      }
      setServoAngle(90 + (int8_t)u); // Attempt to set servo angle
      
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
        setMotor(PWM_NORMAL); // Set motor to normal speed
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
        encoderCount = 0;
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
}