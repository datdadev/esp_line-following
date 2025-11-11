#include "background_tasks.h"
#include <Arduino.h>
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Handle for the LED blinking task
static TaskHandle_t ledBlinkTaskHandle = NULL;
static TaskHandle_t sonarTaskHandle = NULL;

// External variable for sonar distance
extern float sonarDistance;

// Include the function to update sonar distance
extern void updateSonarDistance();

// Task function for blinking the LED
void ledBlinkTask(void *parameter) {
    bool ledState = false;
    while(true) {
        ledState = !ledState;  // Toggle LED state
        digitalWrite(LED_BLUE, ledState);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second (1000ms)
    }
}

// Task function for reading sonar distance in the background
void sonarTask(void *parameter) {
    while(true) {
        // Update the sonar distance in the background
        updateSonarDistance();
        // Delay between readings (100ms = 10Hz update rate) - slower rate helps with motor noise
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void initBackgroundTasks() {
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE, LOW);  // Initialize LED to OFF
}

void startLedBlinkTask(uint8_t core_id) {
    // Create LED blinking task on the specified core
    xTaskCreatePinnedToCore(
        ledBlinkTask,           /* Function to implement the task */
        "LED_BLINK_TASK",       /* Name of the task */
        2048,                   /* Stack size in words */
        NULL,                   /* Task input parameter */
        1,                      /* Priority of the task */
        &ledBlinkTaskHandle,    /* Task handle */
        core_id                 /* Core where the task should run */
    );
}

void startSonarTask(uint8_t core_id) {
    // Create sonar reading task on the specified core
    xTaskCreatePinnedToCore(
        sonarTask,              /* Function to implement the task */
        "SONAR_TASK",           /* Name of the task */
        2048,                   /* Stack size in words */
        NULL,                   /* Task input parameter */
        2,                      /* Priority of the task (higher than LED task) */
        &sonarTaskHandle,       /* Task handle */
        core_id                 /* Core where the task should run */
    );
}

void stopLedBlinkTask() {
    if (ledBlinkTaskHandle != NULL) {
        vTaskDelete(ledBlinkTaskHandle);
        ledBlinkTaskHandle = NULL;
    }
}

void startCustomBackgroundTask(void (*taskFunc)(void*), const char* taskName, uint32_t stackSize, UBaseType_t priority, uint8_t core_id) {
    xTaskCreatePinnedToCore(
        taskFunc,               /* Function to implement the task */
        taskName,               /* Name of the task */
        stackSize,              /* Stack size in words */
        NULL,                   /* Task input parameter */
        priority,               /* Priority of the task */
        NULL,                   /* Task handle */
        core_id                 /* Core where the task should run */
    );
}