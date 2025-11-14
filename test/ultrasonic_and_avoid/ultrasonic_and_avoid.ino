#include <Arduino.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ===================== SYSTEM CONFIGURATION =====================
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

// ===================== OBSTACLE DETECTION & AVOIDANCE PARAMETERS =====================
constexpr uint16_t SONAR_THRESHOLD_OBSTACLE = 450;   // mm - Distance threshold to trigger avoidance
constexpr float OBSTACLE_AVOID_S1 = 264.28;  // mm
constexpr float OBSTACLE_AVOID_S2 = OBSTACLE_AVOID_S1 + 528.56;  // mm  
constexpr float OBSTACLE_AVOID_S3 = OBSTACLE_AVOID_S2 + 264.28;  // mm

// Servo angles for obstacle avoidance
constexpr int8_t SERVO_AVOID_LEFT = 113;
constexpr int8_t SERVO_AVOID_RIGHT = 67;
constexpr int8_t SERVO_STRAIGHT = 90;

// ===================== PIN DEFINITIONS =====================
#define ULTRASONIC_PIN 2     // Ultrasonic sensor pin
#define AIN1    12           // Motor control
#define AIN2    13           // Motor control  
#define PWMA    14           // Motor PWM
#define ENCA    35           // Encoder A
#define ENCB    34           // Encoder B
#define LED_RED 15           // Status LED
#define BTN_BOOT 0           // Boot button
#define SERVO_PIN 16         // Servo control pin

// ===================== SYSTEM STATES =====================
enum SystemState {
  IDLE,
  NORMAL_OPERATION,
  AVOID_PATH,
  COMPLETED
};

SystemState currentState = NORMAL_OPERATION; // Start in normal operation
bool avoidPathActive = false;
float totalDistanceTraveled = 0.0;
float distanceTraveled = 0.0;
int32_t initialEncoderCount = 0;

// ===================== PID PARAMETERS =====================
float TARGET_SPEED = 1.0;     // Reduced target speed for safety (0.5 m/s)
float KP = 150.0;             // Proportional gain
float KI = 1000.0;            // Integral gain  
float KD = 5.0;               // Derivative gain
float INTEGRAL_LIMIT = 100.0; // Anti-windup limit

// ===================== GLOBAL VARIABLES =====================
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

// ===================== ULTRASONIC VARIABLES =====================
float sonarDistance = 0.0;  // Global sonar distance reading
bool obstacleDetected = false;

// ===================== RTOS HANDLES =====================
TaskHandle_t ultrasonicTaskHandle = NULL;
QueueHandle_t sonarQueue = NULL;

// ===================== PID VARIABLES =====================
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pidOutput = 0.0;

// ===================== FUNCTION DECLARATIONS =====================
void IRAM_ATTR encoderISR();
float calculateSpeed();
float calculateDistanceTraveled();
void setServoAngle(uint8_t angle);
void resetPathEncoder();
float computePID(float currentSpeed, float targetSpeed);
void setMotor(int16_t pwm);
void updateAvoidPathState();
int8_t getSonarDistance(float* distance);
void ultrasonicTask(void *parameter);
bool checkForObstacle();

// ===================== ULTRASONIC SENSOR FUNCTIONS =====================
int8_t getSonarDistance(float* distance) {
    static float lastDistance = 800.0f; // For simple low-pass filter
    static bool firstReading = true;    // Flag to handle first reading specially
    const float alpha = 0.25f;          // Filter coefficient
    const float maxDistance = 800.0f;   // Max distance in mm

    if (!distance) return -1; // ERROR_INVALID_PARAMETER

    // Trigger pulse
    pinMode(ULTRASONIC_PIN, OUTPUT);
    digitalWrite(ULTRASONIC_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_PIN, LOW);

    // Read echo
    pinMode(ULTRASONIC_PIN, INPUT);
    uint32_t duration = pulseIn(ULTRASONIC_PIN, HIGH, 30000); // Timeout 30ms (~5m)

    if (duration == 0) {
        // Timeout occurred
        *distance = maxDistance;
        return -2; // ERROR_ULTRASONIC_TIMEOUT
    }

    // Convert to mm (speed of sound ~340 m/s = 0.34 mm/us)
    // Divide by 2 for round trip, so 0.34/2 = 0.17 mm/us
    float newDistance = duration * 0.17f;

    // Clamp maximum distance
    if (newDistance > maxDistance) newDistance = maxDistance;

    // Apply low-pass filter
    if (firstReading) {
        lastDistance = newDistance;
        firstReading = false;
    } else {
        lastDistance = alpha * newDistance + (1 - alpha) * lastDistance;
    }
    
    *distance = lastDistance;
    return 0; // ERROR_SUCCESS
}

// ===================== ULTRASONIC TASK (RUNS ON SEPARATE CORE) =====================
void ultrasonicTask(void *parameter) {
    float distance = 0.0;
    
    Serial.println("Ultrasonic task started on core: " + String(xPortGetCoreID()));
    
    for(;;) {
        // Read ultrasonic sensor
        int8_t result = getSonarDistance(&distance);
        
        // Send data to main task via queue (non-blocking)
        if (sonarQueue != NULL) {
            xQueueOverwrite(sonarQueue, &distance);
        }
        
        // Task delay - adjust frequency as needed (20Hz = 50ms)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===================== OBSTACLE DETECTION FUNCTION =====================
bool checkForObstacle() {
    float currentDistance = sonarDistance;
    bool obstacle = (currentDistance > 0 && currentDistance < SONAR_THRESHOLD_OBSTACLE);
    
    // Visual feedback
    digitalWrite(LED_RED, obstacle ? HIGH : LOW); // LED ON when obstacle detected
    
    return obstacle;
}

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

// ===================== DISTANCE CALCULATION =====================
float calculateDistanceTraveled() {
  int32_t currentCount = encoderCount;
  float wheelRev = (currentCount - initialEncoderCount) / COUNTS_PER_REV;
  float distance = wheelRev * (WHEEL_DIAMETER * 3.14159265) * 1000; // Distance in mm
  return distance;
}

// ===================== SERVO CONTROL =====================
void setServoAngle(uint8_t angle) {
  // Simple PWM servo control - adjust pulse width for your servo
  uint16_t pulseWidth = map(angle, 0, 180, 500, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
  delay(15); // Wait for servo to reach position
}

// ===================== RESET ENCODER FOR PATH =====================
void resetPathEncoder() {
  initialEncoderCount = encoderCount;
  distanceTraveled = 0.0;
}

// ===================== PID CONTROLLER =====================
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

// ===================== MOTOR CONTROL =====================
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

// ===================== OBSTACLE AVOIDANCE STATE MACHINE =====================
void updateAvoidPathState() {
  if (currentState != AVOID_PATH) return;
  
  distanceTraveled = calculateDistanceTraveled();
  
  // Initialize path on first entry
  if (!avoidPathActive) {
    setServoAngle(SERVO_AVOID_LEFT);  // Start turning left
    resetPathEncoder();               // Reset distance measurement
    avoidPathActive = true;
    Serial.println("Avoidance started - Stage 1: Turn left");
  }
  
  // State transitions based on distance
  if (distanceTraveled < OBSTACLE_AVOID_S1) {
    // Stage 1: Turn left
    setServoAngle(SERVO_AVOID_LEFT);
  } else if (distanceTraveled < OBSTACLE_AVOID_S2) {
    // Stage 2: Turn right
    setServoAngle(SERVO_AVOID_RIGHT);
    if (distanceTraveled < OBSTACLE_AVOID_S1 + 10) {
      Serial.println("Stage 2: Turn right");
    }
  } else if (distanceTraveled < OBSTACLE_AVOID_S3) {
    // Stage 3: Turn left again
    setServoAngle(SERVO_AVOID_LEFT);
    if (distanceTraveled < OBSTACLE_AVOID_S2 + 10) {
      Serial.println("Stage 3: Turn left");
    }
  } else {
    // Path completed - return to normal operation
    currentState = NORMAL_OPERATION;
    avoidPathActive = false;
    setServoAngle(SERVO_STRAIGHT); // Go straight
    Serial.println("Avoidance completed - Returning to normal operation");
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(ULTRASONIC_PIN, INPUT);

  // Use external pull-up resistors for encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR, CHANGE);

  // Initialize motor and servo
  setMotor(0);
  setServoAngle(SERVO_STRAIGHT); // Start with straight steering
  
  // Initialize variables
  lastSpeedCalcTime = millis();
  lastEncoderCount = encoderCount;
  initialEncoderCount = encoderCount;
  
  // Initialize PID variables
  error = 0.0;
  lastError = 0.0;
  integral = 0.0;
  derivative = 0.0;
  
  // Create queue for sonar data
  sonarQueue = xQueueCreate(1, sizeof(float));
  
  if (sonarQueue == NULL) {
    Serial.println("Error creating sonar queue!");
  }
  
  // Create ultrasonic task on core 0
  xTaskCreatePinnedToCore(
      ultrasonicTask,        // Task function
      "UltrasonicTask",      // Task name
      4096,                 // Stack size (bytes)
      NULL,                 // Parameter
      1,                    // Priority
      &ultrasonicTaskHandle,// Task handle
      0                     // Core number (0)
  );
  
  // Verify task creation
  if (ultrasonicTaskHandle == NULL) {
    Serial.println("Error creating ultrasonic task!");
  } else {
    Serial.println("Ultrasonic task created successfully");
  }

  Serial.println("Setup completed - System ready");
  Serial.println("State: NORMAL_OPERATION - Moving forward");
  digitalWrite(LED_RED, LOW); // LED off initially
}

// ===================== MAIN LOOP =====================
void loop() {
  static uint32_t lastPIDTime = 0;
  static uint32_t lastPrintTime = 0;
  static bool lastObstacleState = false;
  uint32_t currentTime = millis();
  
  // Receive sonar data from queue (non-blocking)
  float newDistance;
  if (xQueueReceive(sonarQueue, &newDistance, 0) == pdTRUE) {
    sonarDistance = newDistance;
  }
  
  // Check for obstacles
  obstacleDetected = checkForObstacle();
  
  // State transition: Obstacle detected in normal operation
  if (currentState == NORMAL_OPERATION && obstacleDetected && !lastObstacleState) {
    currentState = AVOID_PATH;
    avoidPathActive = false; // Will be activated in state machine
    Serial.println("OBSTACLE DETECTED! Starting avoidance maneuver");
    Serial.println("Distance: " + String(sonarDistance) + " mm");
  }
  
  lastObstacleState = obstacleDetected;

  // Button control for manual motor toggle
  bool buttonState = digitalRead(BTN_BOOT);
  if (buttonState == LOW && lastButtonState == HIGH) {
    motorOn = !motorOn;
    digitalWrite(LED_RED, motorOn ? (obstacleDetected ? HIGH : LOW) : LOW);
    
    if (motorOn) {
      integral = 0.0;
      lastError = 0.0;
      if (currentState == COMPLETED) {
        currentState = NORMAL_OPERATION;
      }
    } else {
      currentState = IDLE;
      avoidPathActive = false;
      setServoAngle(SERVO_STRAIGHT);
    }
    Serial.println("Motor: " + String(motorOn ? "ON" : "OFF"));
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
    } else if (currentState == NORMAL_OPERATION) {
      // Ensure straight steering in normal operation
      setServoAngle(SERVO_STRAIGHT);
    }
    
    lastPIDTime = currentTime;
  } else if (!motorOn) {
    // Stop motor when disabled
    setMotor(0);
    motorPWM = 0;
    integral = 0.0;
  }

  // Print status periodically (every 200ms)
  if (currentTime - lastPrintTime > 200) {
    Serial.printf("State: %s, Dist: %4.0fmm, Speed: %4.2fm/s, PWM: %3d, Obstacle: %s\n",
                  currentState == NORMAL_OPERATION ? "NORMAL" : 
                  currentState == AVOID_PATH ? "AVOIDING" : "IDLE",
                  sonarDistance, speed, motorPWM, 
                  obstacleDetected ? "YES" : "NO");
    lastPrintTime = currentTime;
  }

  delay(10); // Small delay for stability
}