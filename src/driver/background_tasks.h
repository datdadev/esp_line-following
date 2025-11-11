#ifndef BACKGROUND_TASKS_H
#define BACKGROUND_TASKS_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"

/**
 * @brief Initialize the background tasks
 */
void initBackgroundTasks();

/**
 * @brief Start the LED blinking task on a separate core
 * @param core_id The core ID to run the task on (0 or 1)
 */
void startLedBlinkTask(uint8_t core_id);

/**
 * @brief Start the sonar reading task on a separate core
 * @param core_id The core ID to run the task on (0 or 1)
 */
void startSonarTask(uint8_t core_id);

/**
 * @brief Stop the LED blinking task
 */
void stopLedBlinkTask();

/**
 * @brief Start a custom background task on a separate core
 * @param taskFunc Function pointer to the task function
 * @param taskName Name of the task
 * @param stackSize Size of the stack for the task
 * @param priority Priority of the task
 * @param core_id The core ID to run the task on (0 or 1)
 */
void startCustomBackgroundTask(void (*taskFunc)(void*), const char* taskName, uint32_t stackSize, UBaseType_t priority, uint8_t core_id);

#endif // BACKGROUND_TASKS_H