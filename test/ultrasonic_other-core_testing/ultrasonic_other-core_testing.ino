#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ===================== PIN DEFINITIONS =====================
#define ULTRASONIC_PIN 2  // Change this to your actual ultrasonic pin
#define LED_RED 15        // Example LED pin for obstacle indication

// ===================== THRESHOLD CONSTANTS =====================
const uint16_t SONAR_THRESHOLD_OBSTACLE = 450;   // mm

// ===================== GLOBAL VARIABLES =====================
float sonarDistance = 0.0;  // Global sonar distance reading

// ===================== RTOS HANDLES =====================
TaskHandle_t ultrasonicTaskHandle = NULL;
QueueHandle_t sonarQueue = NULL;

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

    // Convert to cm (speed of sound ~340 m/s = 0.034 cm/us)
    // Divide by 2 for round trip, so 0.034/2 = 0.017 cm/us
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
bool detectObstacle() {
    float currentDistance = sonarDistance;
    bool obstacleDetected = (currentDistance > 0 && currentDistance < SONAR_THRESHOLD_OBSTACLE);
    
    // Visual feedback
    digitalWrite(LED_RED, obstacleDetected ? LOW : HIGH);
    
    return obstacleDetected;
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH); // Turn off LED initially
    
    // Create queue for sonar data (size of 1 float)
    sonarQueue = xQueueCreate(1, sizeof(float));
    
    if (sonarQueue == NULL) {
        Serial.println("Error creating sonar queue!");
    }
    
    // Create ultrasonic task on core 0 (leaving core 1 for main loop)
    xTaskCreatePinnedToCore(
        ultrasonicTask,        // Task function
        "UltrasonicTask",      // Task name
        4096,                 // Stack size (bytes)
        NULL,                 // Parameter
        1,                    // Priority (same as main loop)
        &ultrasonicTaskHandle,// Task handle
        0                     // Core number (0 or 1)
    );
    
    // Verify task creation
    if (ultrasonicTaskHandle == NULL) {
        Serial.println("Error creating ultrasonic task!");
    } else {
        Serial.println("Ultrasonic task created successfully");
    }
    
    Serial.println("Setup completed");
    delay(1000);
}

// ===================== MAIN LOOP (RUNS ON CORE 1) =====================
void loop() {
    static uint32_t lastPrintTime = 0;
    
    // Receive sonar data from queue (non-blocking)
    float newDistance;
    if (xQueueReceive(sonarQueue, &newDistance, 0) == pdTRUE) {
        sonarDistance = newDistance;
    }
    
    // Check for obstacles
    bool obstacle = detectObstacle();
    
    // Print status periodically
    if (millis() - lastPrintTime > 100) {
        Serial.println("Sonar: " + String(sonarDistance) + " cm, Obstacle: " + 
                      String(obstacle ? "YES" : "NO") + 
                      ", Core: " + String(xPortGetCoreID()));
        lastPrintTime = millis();
    }
    
    // Your other main loop code here (motor control, line following, etc.)
    
    delay(10); // Small delay to prevent watchdog issues
}