#include <Arduino.h>
#include <ESP32Servo.h>
#include <stdint.h>
#include "../../Config/system_config.h"
#include "../../Inc/pins.h"
#include "../Driver/Devices/motor_driver.h"
#include "../Driver/Components/sensor_driver.h"
#include "../Driver/Components/servo_driver.h"
#include "../MiddleWare/pid_controller.h"
#include "soft_start.h"

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

  initServo();
  setServoAngle(90);

  state = INIT;
  Serial.println("System INIT");
}

// ===================== MAIN LOOP =====================
void loop() {
  if (millis() - lastSample < Ts * 1000) return;
  lastSample = millis();

  readLineSensors();

  switch (state) {

    case INIT:
      setMotor(0);
      setServoAngle(90);
      if (digitalRead(BTN_BOOT) == LOW) {
        state = LINE_FOLLOW;
        first_run_after_init = true;  // Enable soft start for the first run
        Serial.println("Start line following");
      }
      break;

    case LINE_FOLLOW: {
      float e = computeError();
      float u = PID(e);
      setServoAngle(90 + (int8_t)u);
      
      // Use soft start when first entering LINE_FOLLOW state
      if (first_run_after_init) {
        if (!soft_start_active) {
          soft_start_active = true;
          soft_start_start_time = millis();
          first_run_after_init = false;  // Only do soft-start once
        }
        setMotor(getSoftStartSpeed(PWM_NORMAL));
      } else {
        setMotor(PWM_NORMAL);
      }

      if (detectLostLine()) {
        state = LOST_LINE;
        Serial.println("Line lost!");
      } else if (detectObstacle()) {
        state = AVOID_PREPARE;
        Serial.println("Obstacle detected!");
      } else if (detectJunction() && !afterJunction) {
        state = TURN_LEFT_PREPARE;
        Serial.println("Junction ahead!");
      } else if (afterJunction && nearGoal()) {
        state = SLOW_DOWN;
        Serial.println("Near goal!");
      }
      break;
    }

    case AVOID_PREPARE:
      if (avoid_prepare_start == 0) {
        setMotor(PWM_SLOW);
        setServoAngle(130); // steer right
        avoid_prepare_start = millis();
      }
      if (millis() - avoid_prepare_start >= 500) {
        avoid_prepare_start = 0; // reset timer
        state = AVOID_PATH;
      }
      break;

    case AVOID_PATH:
      setMotor(PWM_NORMAL);
      setServoAngle(130);
      if (!detectObstacle()) {
        state = MERGE_SEARCH;
        Serial.println("Obstacle cleared");
      }
      break;

    case MERGE_SEARCH:
      setServoAngle(70); // steer back left to find line
      if (detectJunction() || computeError() != 0) {
        state = LINE_FOLLOW;
        first_run_after_init = false;  // Don't use soft start when resuming from merge
        Serial.println("Merged back to line");
      }
      break;

    case TURN_LEFT_PREPARE:
      if (turn_left_prepare_start == 0) {
        setMotor(PWM_SLOW);
        setServoAngle(50); // left turn
        turn_left_prepare_start = millis();
      }
      if (millis() - turn_left_prepare_start >= 800) {
        turn_left_prepare_start = 0; // reset timer
        afterJunction = true;
        encoderCount = 0;
        setServoAngle(90);
        state = LINE_FOLLOW;
        first_run_after_init = false;  // Don't use soft start when resuming from turn
      }
      break;

    case SLOW_DOWN:
      setMotor(PWM_SLOW);
      setServoAngle(90);
      if (detectFinishLine()) {
        state = STOP;
        Serial.println("Finish line detected!");
      }
      break;

    case STOP:
      setMotor(0);
      setServoAngle(90);
      Serial.println("STOP: Reached goal");
      while (1);
      break;

    case LOST_LINE:
      setMotor(0); // Stop the motor
      setServoAngle(90); // Center the steering
      Serial.println("Lost line! Please reposition robot on track.");
      // Optionally, you can add a small forward movement or steering to try to find the line
      // For now, we'll just stop and wait for manual intervention
      break;
  }
}