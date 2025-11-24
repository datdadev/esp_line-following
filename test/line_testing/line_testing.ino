#include <Arduino.h>

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

// ===================== CONSTANTS =====================
constexpr float Ts = 0.01f;  // 0.01s sampling time
constexpr uint32_t SAMPLE_INTERVAL = 1;  // 10ms in milliseconds

// Lyapunov Controller Constants - SỬA CÁC BIẾN NÀY ĐỂ GIẢM OSCILLATION
constexpr float LYP_K2 = 0.2f;        // GIẢM MẠNH từ 0.8f (giảm P gain)
constexpr float LYP_K3 = 0.3f;        // GIẢM MẠNH từ 1.2f (giảm D gain)  
constexpr float LYP_VREF = 150.0f;    // Giữ nguyên
constexpr float LYP_OMEGA_REF = 0.0f; // Giữ nguyên
constexpr float LYP_STEERING_LIMIT = 0.7854f;  // GIẢM từ 30° xuống 15° (0.26 rad)
constexpr float LYP_WHEELBASE = 0.1f; // Giữ nguyên

// Motor PWM
constexpr int16_t PWM_NORMAL = 250;

// ===================== SENSOR CALIBRATION =====================
const float LINE_CALIBRATION_BASE = 112.0f;
const float LINE_CALIBRATION_COEFFS[7] = {1.01705f, 1.02579f, 0.97291f, 0.98378f, 1.0038f, 1.00516f, 0.99195f};
const int16_t LINE_CALIBRATION_OFFSETS[7] = {122, 92, 118, 99, 125, 103, 122};

// Error calculation coefficients
const float ERROR_COEFFS[7] = {3.0f, 2.0f, 1.0f, 0.0f, -1.0f, -2.0f, -3.0f};
const float SCALING_FACTOR = 13.0f;

// ===================== GLOBAL VARIABLES =====================
int16_t lineSensor[7];
float lineFollowError = 0.0f;

// Lyapunov Controller state variables
float prevE2Filtered = 0.0f;
float prevDe2Filtered = 0.0f;
float prevGammaDeg = 0.0f;

// Pre-computed constants for performance
constexpr float ALPHA_E2 = 0.05f;
constexpr float ALPHA_DE2 = 0.1f;
constexpr float INV_TS = 1.0f / Ts;
constexpr float MAX_STEP_DEG = 2.5f;
constexpr float SMALL_ANGLE_THRESHOLD = 0.174533f; // ~10 degrees in radians
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

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  setupPins();
  
  Serial.println("Line Following Test Started");
  Serial.println("Sampling Time: 0.001s");
  Serial.println("Ready to follow line...");

  setServoAngle(90);
  delay(1000);
}

// ===================== MAIN LOOP =====================
void loop() {
  static uint32_t lastSampleTime = 0;
  uint32_t currentTime = millis();
  
  // Strict 10ms sampling time
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    
    // Read sensors
    if (readLineSensors() == 0) {
      // Compute line error
      float e = computeError();
      lineFollowError = e;
      
      // Apply Lyapunov controller
      float u = 0.0;
      if (LyapunovController(e, &u) == 0) {
        setServoAngle(90 + (int8_t)u);
      }
      
      // Set constant motor speed
      // setMotor(PWM_NORMAL);
      
      // Debug output every 100ms
      static uint32_t lastDebugTime = 0;
      if (currentTime - lastDebugTime >= 100) {
        lastDebugTime = currentTime;
        Serial.print("Error: ");
        Serial.print(e, 3);
        Serial.print(" | Control: ");
        Serial.print(u, 3);
        Serial.print(" | Sensors: ");
        for (int i = 0; i < 7; i++) {
          Serial.print(lineSensor[i]);
          Serial.print(" ");
        }
        Serial.println();
      }
      
      // Check for lost line
      if (detectLostLine()) {
        Serial.println("LOST LINE DETECTED!");
        // setMotor(0);  // Stop motor
      }
      
      // Check for junction
      if (detectJunction()) {
        Serial.println("JUNCTION DETECTED!");
      }
    }
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
  
  // LED
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);  // Turn off initially
}

int muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};

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
  float de2 = (e2Filtered - prevE2) * INV_TS;
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
  // Simple PWM servo control - adjust pulse width for your servo
  uint16_t pulseWidth = map(angle, 0, 180, 500, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
}

void setMotor(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  
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
  uint8_t count = 0;
  bool sensorStates[7];
  
  for (uint8_t i = 0; i < 7; i++) {
    sensorStates[i] = (lineSensor[i] < 200);
    if (sensorStates[i]) count++;
  }
  
  if (count < 5) return false;
  
  bool centerSpan = (sensorStates[2] || sensorStates[3] || sensorStates[4]);
  bool edgeDetected = (sensorStates[0] || sensorStates[1] || sensorStates[5] || sensorStates[6]);
  
  return centerSpan && edgeDetected;
}