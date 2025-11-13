#include <Arduino.h>
#include <stdint.h>

// ===================== PIN DEFINITIONS =====================
#define IR0_PIN        33  // MUX output for Array 1 (front)
#define MUX_S0         25
#define MUX_S1         26
#define MUX_S2         27
#define SERVO_PIN      16
#define MOTOR_PWM      14
#define MOTOR_IN1      12
#define MOTOR_IN2      13
#define LED_RED        15

// Encoder pins for motor PID
#define ENCA           35
#define ENCB           34

// ===================== CONSTANTS =====================
constexpr float LINE_TS = 0.001f;    // 1ms for line following
constexpr float MOTOR_TS = 0.01f;    // 10ms for motor PID
constexpr uint32_t LINE_INTERVAL = 1;    // 1ms
constexpr uint32_t MOTOR_INTERVAL = 10;  // 10ms

// Motor PID Constants
constexpr float WHEEL_DIAMETER = 0.065f;
constexpr float GEAR_RATIO = 10.0f;
constexpr int PULSES_PER_REV = 11;
constexpr float QUAD_FACTOR = 4.0f;
constexpr float COUNTS_PER_REV = PULSES_PER_REV * GEAR_RATIO * 2.714f * QUAD_FACTOR;
constexpr int16_t MAX_PWM = 255;
constexpr float MAX_SPEED = 1.0f;

// Motor PID Parameters
float TARGET_SPEED = 1.0f;     // 1 m/s target
float KP = 150.0f;             // Proportional gain
float KI = 1000.0f;            // Integral gain  
float KD = 5.0f;               // Derivative gain
float INTEGRAL_LIMIT = 100.0f; // Anti-windup limit

// Lyapunov Controller Constants
constexpr float LYP_K2 = 0.2f;
constexpr float LYP_K3 = 0.3f;  
constexpr float LYP_VREF = 150.0f;
constexpr float LYP_OMEGA_REF = 0.0f;
constexpr float LYP_STEERING_LIMIT = 0.7854f;
constexpr float LYP_WHEELBASE = 0.1f;

// ===================== SENSOR CALIBRATION =====================
const float LINE_CALIBRATION_BASE = 112.0f;
const float LINE_CALIBRATION_COEFFS[7] = {1.01705f, 1.02579f, 0.97291f, 0.98378f, 1.0038f, 1.00516f, 0.99195f};
const int16_t LINE_CALIBRATION_OFFSETS[7] = {122, 92, 118, 99, 125, 103, 122};
const float ERROR_COEFFS[7] = {3.0f, 2.0f, 1.0f, 0.0f, -1.0f, -2.0f, -3.0f};
const float SCALING_FACTOR = 13.0f;

// ===================== GLOBAL VARIABLES =====================
int16_t lineSensor[7];
float lineFollowError = 0.0f;

// Motor PID Variables
volatile int32_t encoderCount = 0;
volatile int8_t lastEncoded = 0;
float motorSpeed = 0.0f;
int16_t motorPWM = 0;
uint32_t lastSpeedCalcTime = 0;
int32_t lastEncoderCount = 0;

// PID Controller Variables
float pidError = 0.0f;
float lastPidError = 0.0f;
float pidIntegral = 0.0f;
float pidDerivative = 0.0f;

// Lyapunov Controller state variables
float prevE2Filtered = 0.0f;
float prevDe2Filtered = 0.0f;
float prevGammaDeg = 0.0f;

// Pre-computed constants for performance
constexpr float ALPHA_E2 = 0.05f;
constexpr float ALPHA_DE2 = 0.1f;
constexpr float INV_LINE_TS = 1.0f / LINE_TS;
constexpr float MAX_STEP_DEG = 2.5f;
constexpr float SMALL_ANGLE_THRESHOLD = 0.174533f;
constexpr float K2_VREF = LYP_K2 * LYP_VREF;
constexpr float L_VREF_RATIO = LYP_WHEELBASE / LYP_VREF;

// ===================== FUNCTION PROTOTYPES =====================
void setupPins();
int16_t readMux(uint8_t channel);
int8_t readLineSensors();
float computeError();
int8_t LyapunovController(float e2, float* result);
void setServoAngle(uint8_t angle);
void setMotor(int16_t pwm);
bool detectLostLine();
bool detectJunction();

// Motor PID Functions
void IRAM_ATTR encoderISR();
float calculateSpeed();
float computePID(float currentSpeed, float targetSpeed);

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  setupPins();
  
  Serial.println("Integrated Line Following with Motor PID Started");
  Serial.println("Line Following: 1ms sampling, Motor PID: 10ms sampling");
  Serial.println("Ready to follow line...");

  setServoAngle(90);
  delay(1000);
}

// ===================== MAIN LOOP =====================
void loop() {
  static uint32_t lastLineTime = 0;
  static uint32_t lastMotorTime = 0;
  static uint32_t lastDebugTime = 0;
  uint32_t currentTime = millis();
  
  // Line Following - 1ms sampling
  if (currentTime - lastLineTime >= LINE_INTERVAL) {
    lastLineTime = currentTime;
    
    // Read sensors
    if (readLineSensors() == 0) {
      // Compute line error
      float e = computeError();
      lineFollowError = e;
      
      // Apply Lyapunov controller
      float u = 0.0f;
      if (LyapunovController(e, &u) == 0) {
        setServoAngle(90 + (int8_t)u);
      }
      
      // Check for lost line
      if (detectLostLine()) {
        Serial.println("LOST LINE DETECTED!");
        setMotor(0);  // Stop motor
      }
      
      // Check for junction
      if (detectJunction()) {
        Serial.println("JUNCTION DETECTED!");
      }
    }
  }
  
  // Motor PID - 10ms sampling
  if (currentTime - lastMotorTime >= MOTOR_INTERVAL) {
    lastMotorTime = currentTime;
    
    // Update speed calculation
    calculateSpeed();
    
    // Compute PID and set motor (only if not lost line)
    if (!detectLostLine()) {
      float pidOutput = computePID(motorSpeed, TARGET_SPEED);
      motorPWM = (int16_t)pidOutput;
      setMotor(motorPWM);
    }
  }
  
  // Debug output every 100ms
  if (currentTime - lastDebugTime >= 100) {
    lastDebugTime = currentTime;
    Serial.print("LineErr: ");
    Serial.print(lineFollowError, 3);
    Serial.print(" | Speed: ");
    Serial.print(motorSpeed, 3);
    Serial.print(" m/s | PWM: ");
    Serial.print(motorPWM);
    Serial.print(" | Sensors: ");
    for (int i = 0; i < 7; i++) {
      Serial.print(lineSensor[i] > 1024 ? "B" : "W");
      Serial.print(" ");
    }
    Serial.println();
  }
}

// ===================== HARDWARE SETUP =====================
void setupPins() {
  // MUX pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  
  // Sensor input
  pinMode(IR0_PIN, INPUT);
  
  // Servo
  pinMode(SERVO_PIN, OUTPUT);
  
  // Motor
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  // Encoder
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  
  // LED
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);  // Turn off initially
  
  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR, CHANGE);
}

int muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};

// ===================== ENCODER ISR =====================
void IRAM_ATTR encoderISR() {
  int8_t MSB = digitalRead(ENCA);
  int8_t LSB = digitalRead(ENCB);
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = (lastEncoded << 2) | encoded;

  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderCount--;
  else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderCount++;

  lastEncoded = encoded;
}

// ===================== SPEED CALCULATION =====================
float calculateSpeed() {
  uint32_t currentTime = millis();
  float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0f;
  if (deltaTime <= 0) return motorSpeed;

  int32_t deltaCount = encoderCount - lastEncoderCount;
  float deltaWheelRev = deltaCount / COUNTS_PER_REV;
  float distance = deltaWheelRev * (WHEEL_DIAMETER * 3.14159265f);
  motorSpeed = distance / deltaTime;

  lastEncoderCount = encoderCount;
  lastSpeedCalcTime = currentTime;

  return motorSpeed;
}

// ===================== PID CONTROLLER =====================
float computePID(float currentSpeed, float targetSpeed) {
  // Calculate error
  pidError = targetSpeed - currentSpeed;
  
  // Proportional term
  float proportional = KP * pidError;
  
  // Integral term with anti-windup
  pidIntegral += pidError * MOTOR_TS;
  if (pidIntegral > INTEGRAL_LIMIT) pidIntegral = INTEGRAL_LIMIT;
  if (pidIntegral < -INTEGRAL_LIMIT) pidIntegral = -INTEGRAL_LIMIT;
  float integralTerm = KI * pidIntegral;
  
  // Derivative term
  pidDerivative = (pidError - lastPidError) / MOTOR_TS;
  float derivativeTerm = KD * pidDerivative;
  
  // Compute PID output
  float output = proportional + integralTerm + derivativeTerm;
  
  // Store error for next iteration
  lastPidError = pidError;
  
  return output;
}

// ===================== MUX READING =====================
int16_t readMux(uint8_t channel) {
  if (channel >= 7) return -1;
  
  for (uint8_t i = 0; i < 3; i++)
    digitalWrite(muxPins[i], (channel >> i) & 1);
  delayMicroseconds(5);
  return analogRead(IR0_PIN);
}

// ===================== SENSOR READING =====================
int8_t readLineSensors() {
  for (uint8_t i = 0; i < 7; i++) {
    int16_t rawValue = readMux(i);
    if (rawValue < 0) return -1;
    
    // Apply calibration
    lineSensor[i] = (int16_t)(LINE_CALIBRATION_BASE + 
                  LINE_CALIBRATION_COEFFS[i] * 
                  (rawValue - LINE_CALIBRATION_OFFSETS[i]));
  }
  return 0;
}

// ===================== ERROR COMPUTATION =====================
float computeError() {
  float num = 0.0f;
  float den = 0.0f;
  uint8_t D = 0;
  
  for (uint8_t i = 0; i < 7; i++) {
    float sensorValue = (float)lineSensor[i];
    num += ERROR_COEFFS[i] * sensorValue;
    den += sensorValue;
    
    if (sensorValue >= 1024.0f) D++;
  }
  
  // Handle special cases
  if (D == 0) return 999.0f;  // All white
  if (D >= 5) return 1000.0f; // Mostly black
  
  return (den != 0.0f) ? (num * SCALING_FACTOR / den) : 0.0f;
}

// ===================== LYAPUNOV CONTROLLER =====================
int8_t LyapunovController(float e2, float* result) {
  if (result == nullptr) return -1;
  
  // Fast low-pass filter
  float e2Filtered = prevE2Filtered + ALPHA_E2 * (e2 - prevE2Filtered);
  prevE2Filtered = e2Filtered;
  
  // Compute derivative
  static float prevE2 = e2Filtered;
  float de2 = (e2Filtered - prevE2) * INV_LINE_TS;
  prevE2 = e2Filtered;
  
  // Filter derivative
  float de2Filtered = prevDe2Filtered + ALPHA_DE2 * (de2 - prevDe2Filtered);
  prevDe2Filtered = de2Filtered;
  
  // Orientation error estimation
  float ratio = de2Filtered / LYP_VREF;
  float e3 = (fabs(ratio) < SMALL_ANGLE_THRESHOLD) ? ratio : atan(ratio);
  
  // Compute angular velocity
  float sin_e3 = (fabs(e3) < SMALL_ANGLE_THRESHOLD) ? e3 : sin(e3);
  float omega = LYP_OMEGA_REF + K2_VREF * e2Filtered + LYP_K3 * sin_e3;
  
  // Steering angle computation
  float omega_ratio = L_VREF_RATIO * omega;
  float gamma = (fabs(omega_ratio) < SMALL_ANGLE_THRESHOLD) ? omega_ratio : atan(omega_ratio);
  
  // Saturation
  gamma = (gamma < -LYP_STEERING_LIMIT) ? -LYP_STEERING_LIMIT : 
          (gamma > LYP_STEERING_LIMIT) ? LYP_STEERING_LIMIT : gamma;
  
  // Convert to degrees and rate limiting
  float gammaDeg = gamma * RAD_TO_DEG;
  float delta = gammaDeg - prevGammaDeg;
  if (delta > MAX_STEP_DEG) gammaDeg = prevGammaDeg + MAX_STEP_DEG;
  else if (delta < -MAX_STEP_DEG) gammaDeg = prevGammaDeg - MAX_STEP_DEG;
  prevGammaDeg = gammaDeg;
  
  *result = -gammaDeg;
  return 0;
}

// ===================== ACTUATOR CONTROL =====================
void setServoAngle(uint8_t angle) {
  uint16_t pulseWidth = map(angle, 0, 180, 500, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
}

void setMotor(int16_t pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  
  if (pwm > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    pwm = -pwm;
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  }
  
  analogWrite(MOTOR_PWM, pwm);
}

// ===================== LINE DETECTION =====================
bool detectLostLine() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 7; i++)
    if (lineSensor[i] < 1024) count++;
  return (count == 7);  // All sensors see white
}

bool detectJunction() {
  bool s[7];
  const bool expected[7] = {true, false, false, true, false, false, true}; // W B B W B B W
  uint8_t mismatch = 0;

  // Convert analog readings to binary (true = white, false = black)
  for (uint8_t i = 0; i < 7; i++) {
    s[i] = (lineSensor[i] >= 200);  // >=200 → white, <200 → black
    if (s[i] != expected[i]) mismatch++;
  }

  // Allow up to 2 mismatches (tolerance)
  return mismatch <= 2;
}

