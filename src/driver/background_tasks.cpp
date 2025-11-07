#include "background_tasks.h"
#include <Arduino.h>
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Handle for the LED blinking task
static TaskHandle_t ledBlinkTaskHandle = NULL;

// Task function for blinking the LED
void ledBlinkTask(void *parameter) {
    bool ledState = false;
    while(true) {
        ledState = !ledState;  // Toggle LED state
        digitalWrite(LED_BLUE, ledState);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second (1000ms)
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