#include <Arduino.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ===================== SYSTEM CONFIGURATION =====================
constexpr float Ts = 0.01f;             
constexpr float WHEEL_DIAMETER = 0.065f; 
constexpr float GEAR_RATIO = 10.0f;     
constexpr int PULSES_PER_REV = 11;     
constexpr float QUAD_FACTOR = 4.0f;     
constexpr float COUNTS_PER_REV = PULSES_PER_REV * GEAR_RATIO * 2.714f * QUAD_FACTOR;

constexpr int16_t MAX_PWM = 255;       
constexpr float MAX_SPEED = 1.0f;       

// ===================== OBSTACLE DETECTION =====================
constexpr uint16_t SONAR_THRESHOLD_OBSTACLE = 450;
constexpr float OBSTACLE_AVOID_S1 = 320.0f; // 264.28f;
constexpr float OBSTACLE_AVOID_S2 = OBSTACLE_AVOID_S1 + 450.0f;// 528.56f;  
constexpr float OBSTACLE_AVOID_S3 = OBSTACLE_AVOID_S2 + 320.0f;// 264.28f;
bool obstacleAlreadyDetected = false;

constexpr int8_t SERVO_STRAIGHT = 95;
constexpr int8_t SERVO_AVOID_LEFT = SERVO_STRAIGHT + 23;
constexpr int8_t SERVO_AVOID_RIGHT = SERVO_STRAIGHT - 23;

// ===================== LINE FOLLOWING CONSTANTS =====================
constexpr float LINE_TS = 0.001f;    // 1ms for line following
constexpr uint32_t LINE_INTERVAL = 1;    // 1ms

// Lyapunov Controller Constants
constexpr float LYP_K2 = 0.2f; // 0.2f;
constexpr float LYP_K3 = 0.3f; // 0.3f;  
constexpr float LYP_VREF = 150.0f; // scaling for derivative/heading term
constexpr float LYP_OMEGA_REF = 0.0f;
constexpr float LYP_STEERING_LIMIT = 0.7854f;
constexpr float LYP_WHEELBASE = 0.1f;

// ===================== SENSOR CALIBRATION =====================
const float LINE_CALIBRATION_BASE = 112.0f;
const float LINE_CALIBRATION_COEFFS[7] = {1.01705f, 1.02579f, 0.97291f, 0.98378f, 1.0038f, 1.00516f, 0.99195f};
const int16_t LINE_CALIBRATION_OFFSETS[7] = {122, 92, 118, 99, 125, 103, 122};
const float ERROR_COEFFS[7] = {3.0f, 2.0f, 1.0f, 0.0f, -1.0f, -2.0f, -3.0f};
const float SCALING_FACTOR = 13.0f; // 13.0f
const float ERROR_OFFSET = 0.0f; // 0.0f

// ===================== PIN DEFINITIONS =====================
#define AIN1    12
#define AIN2    13  
#define PWMA    14
#define ENCA    35
#define ENCB    34
#define IR0_PIN 33
#define MUX_S0  25
#define MUX_S1  26
#define MUX_S2  27
#define SERVO_PIN 16
#define LED_RED 15
#define LED_BLUE 17
#define BTN_BOOT 0
#define ULTRASONIC_PIN 2

#define SPEED 1.0f

// ===================== SYSTEM STATES =====================
enum SystemState {
  IDLE,
  LINE_FOLLOWING,
  AVOID_PATH,
  COMPLETED
};

SystemState currentState = IDLE;
bool avoidPathActive = false;
float distanceTraveled = 0.0f;
int32_t initialEncoderCount = 0;

// ===================== MOTOR CONTROL =====================
float TARGET_SPEED = SPEED;
float KP = 150.0f;
float KI = 1000.0f;  
float KD = 5.0f;
float INTEGRAL_LIMIT = 100.0f;

// ===================== GLOBAL VARIABLES =====================
volatile int32_t encoderCount = 0;
volatile int8_t lastEncoded = 0;
int16_t motorPWM = 0;
float motorSpeed = 0.0f;
uint32_t lastSpeedCalcTime = 0;
int32_t lastEncoderCount = 0;

// Line Following Variables
int16_t lineSensor[7];
float lineFollowError = 0.0f;

// Lyapunov Controller Variables
float prevE2Filtered = 0.0f;
float prevDe2Filtered = 0.0f;
float prevGammaDeg = 0.0f;

// System Control
bool motorOn = false;
bool lastButtonState = HIGH;
bool buttonPressed = false;
uint32_t buttonPressTime = 0;
const uint32_t RESET_HOLD_TIME = 3000;

// Ultrasonic Variables
float sonarDistance = 0.0f;
bool obstacleDetected = false;

bool autoStarted = false;

// ===================== LOGGING VARIABLES =====================
float steeringAngleDeg = 0.0f;   // servo steering angle (deg, relative to straight)
float linearSpeed = 0.0f;        // motor linear speed (m/s)
uint8_t currentServoCmd = SERVO_STRAIGHT;


// ===================== RTOS HANDLES =====================
TaskHandle_t ultrasonicTaskHandle = NULL;
QueueHandle_t sonarQueue = NULL;

// PID Variables
float error = 0.0f;
float lastError = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;

// ===================== JUNCTION VARIABLES =====================
bool junction_2 = false;
uint32_t junctionStartTime = 0;
const uint32_t JUNCTION_DURATION = 50; // Giữ trạng thái junction trong 1s

int count = 0;
bool _finished = false;
uint32_t finishDetectedTime = 0;
const uint32_t FINISH_DELAY = 1500; // 1.5s delay before decel

// ===================== FUNCTION DECLARATIONS =====================
void handleButtonPress();
void resetSystem();
void resetPID();
void IRAM_ATTR encoderISR();
float calculateSpeed();
void setServoAngle(uint8_t angle);
void setMotor(int16_t pwm);
float computePID(float currentSpeed, float targetSpeed);

// Line Following Functions
int16_t readMux(uint8_t channel);
int8_t readLineSensors();
float computeError();
int8_t LyapunovController(float e2, float* result);
bool detectLostLine();

// Obstacle Avoidance Functions
void resetPathEncoder();
void updateAvoidPathState();
int8_t getSonarDistance(float* distance);
void ultrasonicTask(void *parameter);
bool checkForObstacle();

// ===================== PID RESET FUNCTION =====================
void resetPID() {
    motorPWM = 0;
    integral = 0.0f;
    lastError = 0.0f;
    error = 0.0f;
    derivative = 0.0f;
    setMotor(0);
}

// ===================== ULTRASONIC FUNCTIONS =====================
int8_t getSonarDistance(float* distance) {
    static float lastDistance = 800.0f;
    static bool firstReading = true;
    const float alpha = 0.25f;
    const float maxDistance = 800.0f;

    if (!distance) return -1;

    pinMode(ULTRASONIC_PIN, OUTPUT);
    digitalWrite(ULTRASONIC_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_PIN, LOW);

    pinMode(ULTRASONIC_PIN, INPUT);
    uint32_t duration = pulseIn(ULTRASONIC_PIN, HIGH, 30000);

    if (duration == 0) {
        *distance = maxDistance;
        return -2;
    }

    float newDistance = duration * 0.17f;
    if (newDistance > maxDistance) newDistance = maxDistance;

    if (firstReading) {
        lastDistance = newDistance;
        firstReading = false;
    } else {
        lastDistance = alpha * newDistance + (1 - alpha) * lastDistance;
    }
    
    *distance = lastDistance;
    return 0;
}

void ultrasonicTask(void *parameter) {
    float distance = 0.0f;
    for(;;) {
        getSonarDistance(&distance);
        if (sonarQueue != NULL) {
            xQueueOverwrite(sonarQueue, &distance);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===================== BUTTON HANDLING =====================
void handleButtonPress() {
    bool currentButtonState = digitalRead(BTN_BOOT);
    uint32_t currentTime = millis();
    
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressTime = currentTime;
        buttonPressed = true;
    }
    
    if (currentButtonState == HIGH && lastButtonState == LOW && buttonPressed) {
        buttonPressed = false;
        uint32_t pressDuration = currentTime - buttonPressTime;
        
        if (pressDuration < 1000) {
            if (currentState == IDLE || currentState == COMPLETED) {
                currentState = LINE_FOLLOWING;
                motorOn = true;
                resetPID();
                setServoAngle(SERVO_STRAIGHT);
                // serial.println("STARTED - Line Following");
            } else {
                currentState = IDLE;
                motorOn = false;
                avoidPathActive = false;
                resetPID();
                setServoAngle(SERVO_STRAIGHT);
                // serial.println("STOPPED");
            }
        } else if (pressDuration >= RESET_HOLD_TIME) {
            resetSystem();
        }
    }
    
    if (buttonPressed && currentButtonState == LOW) {
        uint32_t pressDuration = currentTime - buttonPressTime;
        if (pressDuration >= RESET_HOLD_TIME) {
            resetSystem();
            buttonPressed = false;
        }
    }
    
    lastButtonState = currentButtonState;
}

void resetSystem() {
    // serial.println("SYSTEM RESET");
    currentState = IDLE;
    motorOn = false;
    avoidPathActive = false;
    
    resetPID();
    setServoAngle(SERVO_STRAIGHT);
    
    encoderCount = 0;
    lastEncoderCount = 0;
    initialEncoderCount = 0;
    motorSpeed = 0.0f;
    
    lineFollowError = 0.0f;
    prevE2Filtered = 0.0f;
    prevDe2Filtered = 0.0f;
    prevGammaDeg = 0.0f;
    
    distanceTraveled = 0.0f;
    lastSpeedCalcTime = millis();
    _finished = false;

    junction_2 = false;
    obstacleAlreadyDetected = false;
    
    for(int i = 0; i < 3; i++) {
        digitalWrite(LED_RED, HIGH);
        delay(200);
        digitalWrite(LED_RED, LOW);
        delay(200);
    }
    // serial.println("Reset complete - Press BOOT to start");
}

// ===================== CORE FUNCTIONS =====================
void IRAM_ATTR encoderISR() {
    int8_t MSB = digitalRead(ENCA);
    int8_t LSB = digitalRead(ENCB);
    int8_t encoded = (MSB << 1) | LSB;
    int8_t sum = (lastEncoded << 2) | encoded;

    if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderCount--;
    else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderCount++;

    lastEncoded = encoded;
}

// float calculateSpeed() {
//     uint32_t currentTime = millis();
//     float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0f;
//     if (deltaTime <= 0) return motorSpeed;

//     int32_t deltaCount = encoderCount - lastEncoderCount;
//     float deltaWheelRev = deltaCount / COUNTS_PER_REV;
//     float distance = deltaWheelRev * (WHEEL_DIAMETER * 3.14159265f);
//     motorSpeed = distance / deltaTime;

//     lastEncoderCount = encoderCount;
//     lastSpeedCalcTime = currentTime;

//     return motorSpeed;
// }

float calculateSpeed() {
    uint32_t currentTime = millis();
    float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0f;
    if (deltaTime <= 0) return motorSpeed;

    int32_t deltaCount = encoderCount - lastEncoderCount;
    float deltaWheelRev = deltaCount / COUNTS_PER_REV;
    float distance = deltaWheelRev * (WHEEL_DIAMETER * 3.14159265f); // meters
    motorSpeed = distance / deltaTime; // m/s

    lastEncoderCount = encoderCount;
    lastSpeedCalcTime = currentTime;

    // expose to logger
    linearSpeed = motorSpeed;

    return motorSpeed;
}


// void setServoAngle(uint8_t angle) {
//     angle = constrain(angle, 0, 180);
//     uint16_t pulseWidth = map(angle, 0, 180, 500, 2400);
//     digitalWrite(SERVO_PIN, HIGH);
//     delayMicroseconds(pulseWidth);
//     digitalWrite(SERVO_PIN, LOW);
// }

void setServoAngle(uint8_t angle) {
    angle = constrain(angle, 0, 180);
    currentServoCmd = angle;

    // Convert servo command to steering angle relative to straight position
    // Example: SERVO_STRAIGHT = 95, angle = 118 => +23 deg
    steeringAngleDeg = (float)currentServoCmd - (float)SERVO_STRAIGHT;

    uint16_t pulseWidth = map(angle, 0, 180, 500, 2400);
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);
}


void setMotor(int16_t pwm) {
    // Immediate braking when pwm is 0
    if (pwm == 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, MAX_PWM);  // Full braking power
        return;
    }
    
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
    
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

float computePID(float currentSpeed, float targetSpeed) {
    // Safety check - don't run PID if motor should be off
    if (!motorOn || currentState == IDLE) {
        return 0.0f;
    }
    
    error = targetSpeed - currentSpeed;
    float proportional = KP * error;
    
    integral += error * Ts;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float integralTerm = KI * integral;
    
    derivative = (error - lastError) / Ts;
    float derivativeTerm = KD * derivative;
    
    float output = proportional + integralTerm + derivativeTerm;
    
    // CRITICAL: Clamp output to valid PWM range
    output = constrain(output, -MAX_PWM, MAX_PWM);
    
    lastError = error;
    
    return output;
}

// ===================== LINE FOLLOWING FUNCTIONS =====================
int muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};

int16_t readMux(uint8_t channel) {
    if (channel >= 7) return -1;
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(muxPins[i], (channel >> i) & 1);
    }
    delayMicroseconds(5);
    return analogRead(IR0_PIN);
}

int8_t readLineSensors() {
    for (uint8_t i = 0; i < 7; i++) {
        int16_t rawValue = readMux(i);
        if (rawValue < 0) return -1;
        lineSensor[i] = (int16_t)(LINE_CALIBRATION_BASE + 
                      LINE_CALIBRATION_COEFFS[i] * 
                      (rawValue - LINE_CALIBRATION_OFFSETS[i]));
    }
    return 0;
}

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
    
    // Return 0 for lost line instead of 999 to prevent steering jumps
        digitalWrite(LED_BLUE, HIGH);
    if (D == 0) return 0.0f;
    if (D >= 5) {
        count++;
        digitalWrite(LED_BLUE, LOW);
        junction_2 = true;
        junctionStartTime = millis(); // Ghi nhận thời điểm detect junction
        return 0.0f;
    }
    
    return (den != 0.0f) ? (num * SCALING_FACTOR / den) + ERROR_OFFSET : 0.0f;
}

bool detectLostLine() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < 7; i++) {
        if (lineSensor[i] < 1024) count++;
    }
    return (count == 7);
}

bool detectFullBlack() {
    uint8_t blackCount = 0;
    for (uint8_t i = 0; i < 7; i++) {
        if (lineSensor[i] >= 1024) {
            blackCount++;
        }
    }
    return (blackCount >= 7);
}

bool detectJunction() {
    // Pattern: W B B W B B W (sensor 0-6)
    // Check sensor values for the specific pattern
    bool patternMatch = true;
    
    // Sensor 0: W (White) - giá trị thấp
    if (lineSensor[0] > 500) patternMatch = false;
    
    // Sensor 1,2: B B (Black) - giá trị cao  
    if (lineSensor[1] < 800) patternMatch = false;
    if (lineSensor[2] < 800) patternMatch = false;
    
    // Sensor 3: W (White) - giá trị thấp
    if (lineSensor[3] > 500) patternMatch = false;
    
    // Sensor 4,5: B B (Black) - giá trị cao
    if (lineSensor[4] < 800) patternMatch = false;
    if (lineSensor[5] < 800) patternMatch = false;
    
    // Sensor 6: W (White) - giá trị thấp
    if (lineSensor[6] > 500) patternMatch = false;
    
    return patternMatch;
}

// Lyapunov Controller with proper initialization
int8_t LyapunovController(float e2, float v, float* result) {
    if (result == nullptr) return -1;
    
    constexpr float ALPHA_E2 = 0.05f;
    constexpr float ALPHA_DE2 = 0.1f;
    constexpr float INV_LINE_TS = 1000.0f;
    constexpr float MAX_STEP_DEG = 2.5f;
    constexpr float SMALL_ANGLE_THRESHOLD = 0.174533f;
    constexpr float K2_VREF = LYP_K2 * LYP_VREF;
    constexpr float L_VREF_RATIO = LYP_WHEELBASE / LYP_VREF;
    
    // Initialize on first call
    static bool firstCall = true;
    if (firstCall) {
        prevE2Filtered = 0.0f;
        prevDe2Filtered = 0.0f;
        prevGammaDeg = 0.0f;
        firstCall = false;
    }
    
    // Limit input error to prevent overreaction
    e2 = constrain(e2, -500.0f, 500.0f);
    
    float e2Filtered = prevE2Filtered + ALPHA_E2 * (e2 - prevE2Filtered);
    prevE2Filtered = e2Filtered;
    
    static float prevE2 = e2Filtered;
    float de2 = (e2Filtered - prevE2) * INV_LINE_TS;
    prevE2 = e2Filtered;
    
    // Limit derivative
    de2 = constrain(de2, -1000.0f, 1000.0f);
    
    float de2Filtered = prevDe2Filtered + ALPHA_DE2 * (de2 - prevDe2Filtered);
    prevDe2Filtered = de2Filtered;
    
    // 1) effective speed (avoid divide-by-zero)
    float v_eff = fabs(v);
    if (v_eff < 0.05f) {         // pick something small
        v_eff = 0.05f;           // or use a nominal value
    }

    // 2) heading proxy from derivative
    float ratio = de2Filtered / v_eff;
    float e3 = (fabs(ratio) < SMALL_ANGLE_THRESHOLD) ? ratio : atan(ratio);

    // 3) omega = k2 * v * e2 + k3 * sin(e3)
    float sin_e3 = (fabs(e3) < SMALL_ANGLE_THRESHOLD) ? e3 : sin(e3);
    float omega = LYP_OMEGA_REF + LYP_K2 * v_eff * e2Filtered + LYP_K3 * sin_e3;

    // 4) gamma from bicycle model: gamma = atan(L/v * omega)
    float omega_ratio = (LYP_WHEELBASE / v_eff) * omega;
    float gamma = (fabs(omega_ratio) < SMALL_ANGLE_THRESHOLD) ? omega_ratio : atan(omega_ratio);

    gamma = constrain(gamma, -LYP_STEERING_LIMIT, LYP_STEERING_LIMIT);
    
    float gammaDeg = gamma * 57.2958f;
    
    // Rate limiting
    float delta = gammaDeg - prevGammaDeg;
    if (delta > MAX_STEP_DEG) gammaDeg = prevGammaDeg + MAX_STEP_DEG;
    else if (delta < -MAX_STEP_DEG) gammaDeg = prevGammaDeg - MAX_STEP_DEG;
    
    prevGammaDeg = gammaDeg;
    
    // Limit final output
    gammaDeg = constrain(gammaDeg, -45.0f, 45.0f);
    *result = -gammaDeg;
    
    return 0;
}

// ===================== OBSTACLE FUNCTIONS =====================
bool checkForObstacle() {
    // Nếu đã detect obstacle rồi thì không check nữa
    if (obstacleAlreadyDetected) {
        return false;
    }
    
    float currentDistance = sonarDistance;
    bool obstacle = (currentDistance > 0 && currentDistance < SONAR_THRESHOLD_OBSTACLE);
    
    if (obstacle) {
        obstacleAlreadyDetected = true;  // Đánh dấu đã detect
    }
    
    digitalWrite(LED_RED, (currentState != IDLE && obstacle) ? HIGH : LOW);
    return obstacle;
}

void resetPathEncoder() {
    initialEncoderCount = encoderCount;
    distanceTraveled = 0.0f;
}

float calculateDistanceTraveled() {
    int32_t currentCount = encoderCount;
    float wheelRev = (currentCount - initialEncoderCount) / COUNTS_PER_REV;
    return wheelRev * (WHEEL_DIAMETER * 3.14159265f) * 1000.0f;
}

void updateAvoidPathState() {
    if (currentState != AVOID_PATH) return;
    
    distanceTraveled = calculateDistanceTraveled();
    
    if (!avoidPathActive) {
        setServoAngle(SERVO_AVOID_LEFT);
        resetPathEncoder();
        avoidPathActive = true;
        TARGET_SPEED = SPEED * 0.8f; // Giảm tốc khi bắt đầu tránh vật cản
        // serial.println("Avoidance: Stage 1 - Turn left");
        return;
    }
    
    if (distanceTraveled < OBSTACLE_AVOID_S1) {
        setServoAngle(SERVO_AVOID_LEFT);
        // TARGET_SPEED = 0.3f;
    } 
    else if (distanceTraveled < OBSTACLE_AVOID_S2) {
        setServoAngle(SERVO_AVOID_RIGHT);
        // TARGET_SPEED = 0.3f;
        
        // Chỉ bắt đầu tìm line sau khi đã đi qua 1 đoạn an toàn
        if (distanceTraveled > OBSTACLE_AVOID_S1 + 200.0f) {
            if (readLineSensors() == 0) {
                float e = computeError();
                // Điều kiện nghiêm ngặt hơn để đảm bảo line ổn định
                if (fabs(e) > 0.5f && !detectLostLine()) {
                    startLineRecovery(); // Chuyển sang giai đoạn recovery
                    return;
                }
            }
        }
    } 
    else {
        setServoAngle(SERVO_AVOID_RIGHT);
        TARGET_SPEED = SPEED * 0.5f;
        
        if (readLineSensors() == 0) {
            float e = computeError();
            if (fabs(e) > 0.3f && !detectLostLine()) {
                startLineRecovery();
            }
        }
    }
}

// Hàm chuyển tiếp để ổn định line
void startLineRecovery() {
    // serial.println("Line detected! Starting recovery...");
    
    // Giai đoạn 1: Tiếp tục quay phải nhẹ để căn line
    setServoAngle(SERVO_STRAIGHT + 10); // Quay phải nhẹ
    TARGET_SPEED = SPEED * 0.5f; // Tốc độ chậm
    delay(100); // Chạy 100ms với góc này
    
    // Giai đoạn 2: Chuyển về line following với tốc độ thấp
    currentState = LINE_FOLLOWING;
    avoidPathActive = false;
    TARGET_SPEED = SPEED * 0.5f; // Tốc độ ban đầu thấp
    
    // serial.println("Recovery complete - Back to line following");
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32");
    delay(500);  // Wait for serial to initialize

    // Initialize pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(ENCA, INPUT_PULLUP);
    pinMode(ENCB, INPUT_PULLUP);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(BTN_BOOT, INPUT_PULLUP);
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(ULTRASONIC_PIN, INPUT);
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(IR0_PIN, INPUT);

    // Initialize encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), encoderISR, CHANGE);

    // Initialize motors and servos with stabilization
    setMotor(0);
    
    // Servo stabilization sequence
    Serial.println("Initializing servo...");
    
    // Move to center position and hold
    setServoAngle(SERVO_STRAIGHT);
    delay(500);  // Wait 500ms for servo to reach and stabilize
    
    // Optional: Wiggle test to ensure servo is working
    setServoAngle(SERVO_STRAIGHT - 25);
    delay(200);
    setServoAngle(SERVO_STRAIGHT + 25);
    delay(200);
    setServoAngle(SERVO_STRAIGHT);
    delay(300);  // Final stabilization
    
    Serial.println("Servo initialized and stabilized");
    
    // Initialize variables
    lastSpeedCalcTime = millis();
    lastEncoderCount = encoderCount;
    
    // Create ultrasonic task
    sonarQueue = xQueueCreate(1, sizeof(float));
    if (xTaskCreatePinnedToCore(ultrasonicTask, "UltrasonicTask", 4096, NULL, 1, &ultrasonicTaskHandle, 0) == pdPASS) {
        Serial.println("Ultrasonic task started successfully");
    } else {
        Serial.println("Failed to create ultrasonic task");
    }

    // System ready indication
    for(int i = 0; i < 2; i++) {
        digitalWrite(LED_RED, HIGH);
        delay(100);
        digitalWrite(LED_RED, LOW);
        delay(100);
    }
    
    Serial.println("========================================");
    Serial.println("     SYSTEM READY - BOOT BUTTON CONTROL");
    Serial.println("========================================");
    Serial.println("Short press: Start/Stop");
    Serial.println("Long press (3s): System Reset");
    Serial.println("Current State: IDLE");
    Serial.println("========================================");
}

// ===================== MAIN LOOP =====================
void loop() {
    static uint32_t lastLineTime = 0;
    static uint32_t lastPIDTime = 0;
    static uint32_t lastPrintTime = 0;
    static bool lastObstacleState = false;
    uint32_t currentTime = millis();
    
    // Handle button input
    handleButtonPress();

    // // Auto-start after 1 second
    // static uint32_t startTime = millis();
    // if (!autoStarted && millis() - startTime >= 3000) {
    //     currentState = LINE_FOLLOWING;
    //     motorOn = true;
    //     resetPID();
    //     setServoAngle(SERVO_STRAIGHT);
    //     autoStarted = true;
    // }
    // if (autoStarted) handleButtonPress();
    
    // Get ultrasonic data
    float newDistance;
    if (xQueueReceive(sonarQueue, &newDistance, 0) == pdTRUE) {
        sonarDistance = newDistance;
    }
    
    // HIGH PRIORITY: Line Following (1ms)
    if (currentState == LINE_FOLLOWING && currentTime - lastLineTime >= LINE_INTERVAL) {
        lastLineTime = currentTime;
        
        if (readLineSensors() == 0) {
            float e = computeError();
            lineFollowError = e;
            
            // FIRST: Check for full black (finish line)
            if (detectFullBlack()) {
                setServoAngle(SERVO_STRAIGHT);
                TARGET_SPEED = 0.0f;
                setMotor(0);
                motorOn = false;
                return;
            }
            
            // SECOND: Check for junction phase 2
            if (junction_2) {
                setServoAngle(SERVO_AVOID_RIGHT); 
                
                if (millis() - junctionStartTime > JUNCTION_DURATION) {
                    junction_2 = false;
                    _finished = true;
                    finishDetectedTime = millis();
                }
            }
            // THIRD: Check for lost line
            else if (detectLostLine()) {
                if (_finished) {
                    setServoAngle(SERVO_STRAIGHT);
                    TARGET_SPEED = 0.0f;
                    setMotor(0);
                    currentState = COMPLETED;
                    motorOn = false;
                    return;
                } else {
                    setServoAngle(SERVO_STRAIGHT);
                    TARGET_SPEED = _finished ? 0.0f : SPEED * 0.1f;
                }
            } 
            // FOURTH: Normal line following
            else {
                if (_finished) {
                    uint32_t timeSinceFinish = millis() - finishDetectedTime * SPEED;
                    
                    if (timeSinceFinish >= FINISH_DELAY) {
                        if (TARGET_SPEED > SPEED * 0.2f)
                            TARGET_SPEED -= SPEED * 0.0025f;
                    } else {
                        TARGET_SPEED = SPEED; 
                    }
                } else {
                    if (TARGET_SPEED < SPEED) {
                        TARGET_SPEED += SPEED * 0.01f;
                    }
                }
                
                float steeringCorrection = 0.0f;
                if (LyapunovController(e, linearSpeed, &steeringCorrection) == 0) {
                    setServoAngle(SERVO_STRAIGHT + (int8_t)steeringCorrection);
                }
            }
        }
    }
    
    // Check for obstacles
    if (currentState == LINE_FOLLOWING && !obstacleAlreadyDetected) {  // Thêm điều kiện
        obstacleDetected = checkForObstacle();
        if (obstacleDetected && !lastObstacleState) {
            currentState = AVOID_PATH;
            avoidPathActive = false;
            // Serial.println("Obstacle detected! Switching to avoidance");
        }
        lastObstacleState = obstacleDetected;
    }
    
    // MEDIUM PRIORITY: Motor PID (10ms)
    if (currentTime - lastPIDTime >= 10) {
        lastPIDTime = currentTime;
        
        calculateSpeed();
        
        if (motorOn && currentState != IDLE) {
            float pidOutput = computePID(motorSpeed, TARGET_SPEED);
            motorPWM = (int16_t)pidOutput;
            setMotor(motorPWM);
        } else {
            // Force stop when not running
            resetPID();
        }
        
        // Update avoidance path if active
        if (currentState == AVOID_PATH) {
            // TARGET_SPEED = 0.4f;
            updateAvoidPathState();
        }
    }
    
    // // LOW PRIORITY: Status printing (20ms)
    // if (currentTime - lastPrintTime >= 20) {
    //     const char* stateStr;
    //     switch(currentState) {
    //         case IDLE: stateStr = "IDLE"; break;
    //         case LINE_FOLLOWING: stateStr = "LINE_FOLLOWING"; break;
    //         case AVOID_PATH: stateStr = "AVOIDING"; break;
    //         default: stateStr = "UNKNOWN";
    //     }
        
    //     // SerialBT.printf("State: %-12s Sonar: %4.0fmm LineErr: %6.2f Speed: %4.2fm/s PWM: %3d\n",
    //     //               stateStr, sonarDistance, lineFollowError, motorSpeed, motorPWM);
    //     // SerialBT.printf("%4.2f, %4.2f, %4.2f\n", lineFollowError, steering_angle, linear_speed);
    //     lastPrintTime = currentTime;
    // }

    // LOW PRIORITY: Status printing (20ms)
    if (currentTime - lastPrintTime >= 20) {
        lastPrintTime = currentTime;

        // Make sure latest speed is used (optional, if not already updated in last 10ms PID block)
        // calculateSpeed();  // uncomment if you want strictly 20ms speed update

        // Log: line error, steering angle (deg), linear speed (m/s)
        // Format: e, gamma_deg, v
        SerialBT.printf("%4.2f, %4.2f, %4.3f\n",
                        lineFollowError,
                        steeringAngleDeg,
                        linearSpeed);
    }

    // delay(1);
}