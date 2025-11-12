#include <Arduino.h>
#include <stdint.h>

// ================== system_config.h ==================
constexpr float Ts = 0.01;             
constexpr float WHEEL_DIAMETER = 0.065; 
constexpr float GEAR_RATIO = 10.0;     
constexpr int PULSES_PER_REV = 11;     
constexpr float QUAD_FACTOR = 4.0;     
constexpr float COUNTS_PER_REV = PULSES_PER_REV * GEAR_RATIO * 2.714 * QUAD_FACTOR;

constexpr int16_t MAX_PWM = 255;       
constexpr float MAX_SPEED = 1.0;       
constexpr float SPEED_PER_PWM = MAX_SPEED / MAX_PWM;  

constexpr int16_t PWM_NORMAL = static_cast<int16_t>(1.0 / SPEED_PER_PWM);
constexpr int16_t PWM_SLOW   = static_cast<int16_t>(0.3 / SPEED_PER_PWM);

// ================== pins.h ==================
#define AIN1    12
#define AIN2    13
#define PWMA    14
#define ENCA    35
#define ENCB    34
#define LED_RED 15
#define BTN_BOOT 0

volatile int32_t encoderCount = 0;
volatile int8_t lastEncoded = 0;

int16_t motorPWM = 0;
float speed = 0.0;
uint32_t lastSpeedCalcTime = 0;
int32_t lastEncoderCount = 0;

bool motorOn = false;
bool lastButtonState = HIGH;

float deltaWheelRev = 0.0;
float totalWheelRev = 0.0;

// ================== Encoder ISR ==================
void IRAM_ATTR encoderISR() {
  int8_t MSB = digitalRead(ENCA);
  int8_t LSB = digitalRead(ENCB);
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = (lastEncoded << 2) | encoded;

  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderCount++;
  else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderCount--;

  lastEncoded = encoded;
}

// ================== Speed Calculation ==================
float calculateSpeed() {
  uint32_t currentTime = millis();
  float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0; // seconds
  if (deltaTime <= 0) return speed;

  int32_t deltaCount = encoderCount - lastEncoderCount;
  deltaWheelRev = deltaCount / COUNTS_PER_REV; // revolutions in this period
  totalWheelRev += deltaWheelRev;

  float distance = deltaWheelRev * (WHEEL_DIAMETER * 3.14159265);
  speed = distance / deltaTime;

  lastEncoderCount = encoderCount;
  lastSpeedCalcTime = currentTime;

  return speed;
}

// ================== Motor Control ==================
void setMotor(int16_t pwm) {
  if (pwm >= 0) {
    analogWrite(PWMA, pwm);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    analogWrite(PWMA, -pwm);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("===== Encoder & Motor Test =====");

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(BTN_BOOT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR, CHANGE);

  motorPWM = PWM_NORMAL;
  setMotor(0);
  lastSpeedCalcTime = millis();
  lastEncoderCount = encoderCount;
}

// ================== Loop ==================
void loop() {
  // Toggle motor with button
  bool buttonState = digitalRead(BTN_BOOT);
  if (buttonState == LOW && lastButtonState == HIGH) { // falling edge
    motorOn = !motorOn;
    digitalWrite(LED_RED, motorOn ? HIGH : LOW);
  }
  lastButtonState = buttonState;

  // Run motor
  setMotor(motorOn ? motorPWM : 0);

  // Update speed & wheel revolutions
  calculateSpeed();

  // Print telemetry
  Serial.printf("Encoder: %6ld  PWM: %3d  Speed: %.3f m/s  ΔRev: %.3f  TotalRev: %.3f\n",
                encoderCount, motorOn ? motorPWM : 0, speed, deltaWheelRev, totalWheelRev);

  delay(100);
}
