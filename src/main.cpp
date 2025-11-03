#include <Arduino.h>
#include <ESP32Servo.h>
#include "../include/pins.h"

// ===================== SYSTEM CONSTANTS =====================
constexpr float Ts = 0.01;             // Sampling time (10 ms)
constexpr int NUM_IR = 7;              // 7 TCRT5000 sensors
constexpr float Kp = 0.5;
constexpr float Ki = 0.0;
constexpr float Kd = 0.1;

constexpr int SONAR_TH_OBS = 20;       // cm
constexpr int JUNCTION_COUNT = 5;
constexpr float DIST_NEAR_GOAL = 35.0; // cm after junction
constexpr int PWM_NORMAL = 200;
constexpr int PWM_SLOW = 120;

// ===================== GLOBAL VARIABLES =====================
Servo steering;

enum State {
  INIT, IDLE, LINE_FOLLOW,
  AVOID_PREPARE, AVOID_PATH, MERGE_SEARCH,
  TURN_LEFT_PREPARE, SLOW_DOWN, STOP
};
State state = INIT;

int lineSensor[NUM_IR];
float sonarDistance = 0.0;
long encoderCount = 0;
bool afterJunction = false;
bool nearGoalFlag = false;
unsigned long lastSample = 0;

// ===================== MUX SELECT PINS =====================
const int muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};
const int irPins[2] = {IR1_PIN, IR2_PIN};

// ===================== PID CONTROLLER =====================
float prevE = 0.0, integral = 0.0;

float PID(float e) {
  float derivative = (e - prevE) / Ts;
  integral += e * Ts;
  float u = Kp * e + Ki * integral + Kd * derivative;
  prevE = e;
  return constrain(u, -30, 30); // servo angle offset
}

// ===================== READ MUX CHANNEL =====================
int readMux(int channel, int arrayIdx) {
  for (int i = 0; i < 3; i++)
    digitalWrite(muxPins[i], (channel >> i) & 1);
  delayMicroseconds(5);
  return analogRead(irPins[arrayIdx]);
}

// ===================== READ LINE SENSORS =====================
void readLineSensors() {
  // Suppose 7 sensors are connected via 2 multiplexers (example)
  for (int i = 0; i < NUM_IR; i++) {
    int arrayIdx = (i < 4) ? 0 : 1;
    int ch = (i < 4) ? i : (i - 4);
    lineSensor[i] = readMux(ch, arrayIdx);
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
  long duration = pulseIn(ULTRASONIC_PIN, HIGH, 30000);
  return duration * 0.034 / 2.0;
}

// ===================== DETECTION FUNCTIONS =====================
bool detectObstacle() {
  sonarDistance = getSonarDistance();
  return (sonarDistance > 0 && sonarDistance < SONAR_TH_OBS);
}

bool detectJunction() {
  int count = 0;
  for (int i = 0; i < NUM_IR; i++)
    if (lineSensor[i] < 500) count++; // black threshold
  return (count >= JUNCTION_COUNT);
}

bool detectFinishLine() {
  int count = 0;
  for (int i = 0; i < NUM_IR; i++)
    if (lineSensor[i] < 500) count++;
  return (count >= 6);
}

// ===================== ERROR COMPUTATION =====================
float computeError() {
  int weights[7] = {-3, -2, -1, 0, 1, 2, 3};
  int blackCount = 0;
  int sumW = 0;

  for (int i = 0; i < NUM_IR; i++) {
    if (lineSensor[i] < 500) { // black
      sumW += weights[i];
      blackCount++;
    }
  }

  if (blackCount == 0) return 0;
  return (float)sumW / blackCount;
}

// ===================== MOTOR CONTROL =====================
void setMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm);
  }
}

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
  for (int i = 0; i < 3; i++) pinMode(muxPins[i], OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, RISING);

  steering.attach(SERVO_PIN);
  steering.write(90);

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
      steering.write(90);
      if (digitalRead(BTN_BOOT) == LOW) {
        state = LINE_FOLLOW;
        Serial.println("Start line following");
      }
      break;

    case LINE_FOLLOW: {
      float e = computeError();
      float u = PID(e);
      steering.write(90 + u);
      setMotor(PWM_NORMAL);

      if (detectObstacle()) {
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
      setMotor(PWM_SLOW);
      steering.write(130); // steer right
      delay(500);
      state = AVOID_PATH;
      break;

    case AVOID_PATH:
      setMotor(PWM_NORMAL);
      steering.write(130);
      if (!detectObstacle()) {
        state = MERGE_SEARCH;
        Serial.println("Obstacle cleared");
      }
      break;

    case MERGE_SEARCH:
      steering.write(70); // steer back left to find line
      if (detectJunction() || computeError() != 0) {
        state = LINE_FOLLOW;
        Serial.println("Merged back to line");
      }
      break;

    case TURN_LEFT_PREPARE:
      setMotor(PWM_SLOW);
      steering.write(50); // left turn
      delay(800);
      afterJunction = true;
      encoderCount = 0;
      steering.write(90);
      state = LINE_FOLLOW;
      break;

    case SLOW_DOWN:
      setMotor(PWM_SLOW);
      steering.write(90);
      if (detectFinishLine()) {
        state = STOP;
        Serial.println("Finish line detected!");
      }
      break;

    case STOP:
      setMotor(0);
      steering.write(90);
      Serial.println("STOP: Reached goal");
      while (1);
      break;
  }
}
