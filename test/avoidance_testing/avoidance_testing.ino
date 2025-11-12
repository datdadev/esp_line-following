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

// ================== Obstacle Avoidance Parameters ==================
constexpr float OBSTACLE_AVOID_S1 = 264.28;  // mm
constexpr float OBSTACLE_AVOID_S2 = OBSTACLE_AVOID_S1 + 528.56;  // mm  
constexpr float OBSTACLE_AVOID_S3 = OBSTACLE_AVOID_S2 + 264.28;  // mm

// Servo angles for obstacle avoidance
constexpr int8_t SERVO_AVOID_LEFT = 113.336;
constexpr int8_t SERVO_AVOID_RIGHT = 66.664;

// ================== System States ==================
enum SystemState {
  IDLE,
  AVOID_PATH,
  COMPLETED
};

SystemState currentState = AVOID_PATH; // Start in AVOID_PATH state
bool avoidPathActive = false;
float totalDistanceTraveled = 0.0;
float distanceTraveled = 0.0;
int32_t initialEncoderCount = 0;

// ================== PID Parameters ==================
float TARGET_SPEED = 1.0;     // 1 m/s target
float KP = 150.0;             // Proportional gain - FIXED
float KI = 1000.0;             // Integral gain - FIXED  
float KD = 5.0;               // Derivative gain - FIXED
float INTEGRAL_LIMIT = 100.0; // Anti-windup limit

// ================== pins.h ==================
#define AIN1    12
#define AIN2    13
#define PWMA    14
#define ENCA    35
#define ENCB    34
#define LED_RED 15
#define BTN_BOOT 0
#define SERVO_PIN 16  // Servo control pin

volatile int32_t encoderCount = 0;
volatile int8_t lastEncoded = 0;

int16_t motorPWM = 0;
float speed = 0.0;
uint32_t lastSpeedCalcTime = 0;
int32_t lastEncoderCount = 0;

bool motorOn = true; // Start with motor on
bool lastButtonState = HIGH;

float deltaWheelRev = 0.0;
float totalWheelRev = 0.0;

// ================== PID Variables ==================
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pidOutput = 0.0;

// ================== Serial Command Variables ==================
String inputString = "";
bool stringComplete = false;

// ================== Encoder ISR ==================
void IRAM_ATTR encoderISR() {
  int8_t MSB = digitalRead(ENCA);
  int8_t LSB = digitalRead(ENCB);
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = (lastEncoded << 2) | encoded;

  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderCount--;
  else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderCount++;

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

// ================== Distance Calculation ==================
float calculateDistanceTraveled() {
  int32_t currentCount = encoderCount;
  float wheelRev = (currentCount - initialEncoderCount) / COUNTS_PER_REV;
  float distance = wheelRev * (WHEEL_DIAMETER * 3.14159265) * 1000; // Distance in mm
  return distance;
}

// ================== Servo Control ==================
void setServoAngle(uint8_t angle) {
  // Simple PWM servo control - adjust pulse width for your servo
  uint16_t pulseWidth = map(angle, 0, 180, 500, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
}

// ================== Reset Encoder for Path ==================
void resetPathEncoder() {
  initialEncoderCount = encoderCount;
  distanceTraveled = 0.0;
}

// ================== PID Controller ==================
float computePID(float currentSpeed, float targetSpeed) {
  // Calculate error
  error = targetSpeed - currentSpeed;
  
  // Proportional term
  float proportional = KP * error;
  
  // Integral term with anti-windup
  integral += error * Ts;
  
  // Clamp integral to prevent windup
  if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
  
  float integralTerm = KI * integral;
  
  // Derivative term (filtered)
  derivative = (error - lastError) / Ts;
  float derivativeTerm = KD * derivative;
  
  // Compute PID output
  pidOutput = proportional + integralTerm + derivativeTerm;
  
  // Store error for next iteration
  lastError = error;
  
  return pidOutput;
}

// ================== Motor Control ==================
void setMotor(int16_t pwm) {
  // Apply PWM limits
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  if (pwm < -MAX_PWM) pwm = -MAX_PWM;
  
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

// ================== Obstacle Avoidance State Machine ==================
void updateAvoidPathState() {
  if (currentState != AVOID_PATH) return;
  
  distanceTraveled = calculateDistanceTraveled();
  
  // Initialize path on first entry
  if (!avoidPathActive) {
    setServoAngle(90);  // Steer max left 66.664 degrees
    resetPathEncoder();               // Reset distance measurement
    avoidPathActive = true;
  }
  
  // State transitions based on distance
  if (distanceTraveled < OBSTACLE_AVOID_S1) {
    // Stage 1: Turn left
    setServoAngle(SERVO_AVOID_LEFT);
  } else if (distanceTraveled < OBSTACLE_AVOID_S2) {
    // Stage 2: Turn right
    setServoAngle(SERVO_AVOID_RIGHT);
  } else if (distanceTraveled < OBSTACLE_AVOID_S3) {
    // Stage 3: Turn left again
    setServoAngle(SERVO_AVOID_LEFT);
  } else {
    // Path completed
    currentState = COMPLETED;
    avoidPathActive = false;
    motorOn = false;
    setMotor(0);
  }
}

// ================== Serial Command Processing ==================
void processSerialCommand() {
  if (stringComplete) {
    inputString.trim();
    
    if (inputString.length() > 0) {
      if (inputString == "stop") {
        motorOn = false;
        currentState = IDLE;
        avoidPathActive = false;
        digitalWrite(LED_RED, LOW);
        setMotor(0);
      }
      else if (inputString == "avoid") {
        motorOn = true;
        currentState = AVOID_PATH;
        avoidPathActive = false; // Will be activated in state machine
        digitalWrite(LED_RED, HIGH);
        integral = 0.0;
        lastError = 0.0;
      }
    }
    
    inputString = "";
    stringComplete = false;
  }
}

// ================== Serial Event ==================
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  // Encoder pins
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  
  // LED and button
  pinMode(LED_RED, OUTPUT);
  pinMode(BTN_BOOT, INPUT_PULLUP);
  
  // Servo pin
  pinMode(SERVO_PIN, OUTPUT);

  // Use external pull-up resistors for encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR, CHANGE);

  setMotor(0);
  lastSpeedCalcTime = millis();
  lastEncoderCount = encoderCount;
  initialEncoderCount = encoderCount;
  
  // Initialize PID variables
  error = 0.0;
  lastError = 0.0;
  integral = 0.0;
  derivative = 0.0;
  
  // Start obstacle avoidance immediately
  digitalWrite(LED_RED, HIGH);
  resetPathEncoder();

  setServoAngle(90);
  delay(1000);
}

// ================== Loop ==================
void loop() {
  static uint32_t lastPIDTime = 0;
  uint32_t currentTime = millis();
  
  // Process serial commands (only for stop/avoid now)
  // processSerialCommand();
  
  // Toggle motor with button (fallback)
  bool buttonState = digitalRead(BTN_BOOT);
  if (buttonState == LOW && lastButtonState == HIGH) {
    motorOn = !motorOn;
    digitalWrite(LED_RED, motorOn ? HIGH : LOW);
    
    if (motorOn) {
      integral = 0.0;
      lastError = 0.0;
      if (currentState == COMPLETED) {
        currentState = AVOID_PATH; // Restart avoid path if completed
      }
    } else {
      currentState = IDLE;
      avoidPathActive = false;
    }
  }
  lastButtonState = buttonState;

  // Update speed calculation
  calculateSpeed();

  // Run PID control at fixed intervals (10ms)
  if (motorOn && (currentTime - lastPIDTime >= Ts * 1000)) {
    // Compute PID output for constant speed
    float pidOutput = computePID(speed, TARGET_SPEED);
    motorPWM = (int16_t)pidOutput;
    
    // Apply motor command
    setMotor(motorPWM);
    
    // Update obstacle avoidance state machine
    if (currentState == AVOID_PATH) {
      updateAvoidPathState();
    }
    
    lastPIDTime = currentTime;
  } else if (!motorOn) {
    // Stop motor when disabled
    setMotor(0);
    motorPWM = 0;
    integral = 0.0;
  }

  Serial.printf("%6.4f, %6.4f\n", speed, distanceTraveled);

  delay(10); // Small delay for stability
}