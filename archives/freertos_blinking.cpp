#include <Arduino.h>
#include "../include/pins.h"

// LED Blink Task Function
void ledBlinkTask(void *parameter) {
  while(1) { // Infinite loop
    digitalWrite(LED_BLUE, HIGH);  
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 1000ms (1 sec)  
    digitalWrite(LED_BLUE, LOW);  
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 1000ms (1 sec)
  }
  vTaskDelete(NULL); // Should not reach here
}

void setup() {
  pinMode(LED_BLUE, OUTPUT);
  Serial.begin(115200);
  Serial.println("ESP32-WROOM-32UE: LED Blink Task Running in Background");

  // Create the FreeRTOS task for LED blinking (running independently)
  xTaskCreate(
    ledBlinkTask,   // Task function  
    "LED Blink",    // Task name  
    2000,           // Stack size (bytes)  
    NULL,           // Parameter  
    1,              // Priority (higher = more priority)  
    NULL            // Task handle (not needed here)  
  );

  // Your other setup code (WiFi, ADC, etc.) can go here
}

void loop() {
  // Your main program logic (if any)
  // The LED will blink independently in the background
  vTaskDelay(100); // Small delay if needed
}