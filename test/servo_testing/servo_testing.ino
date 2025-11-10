#include <Arduino.h>
#include <ESP32Servo.h>

// Pin definition
const int servoPin = 16;
Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.setPeriodHertz(50);        // Standard 50 Hz for servo
  myServo.attach(servoPin, 500, 2500); // Min 500us, Max 2500us pulse width
  Serial.println("ESP32 Servo Test on GPIO16");
  Serial.println("Enter an angle between 0 and 180:");
}

void loop() {
  // Check if user typed something in Serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int angle = input.toInt(); // Convert input to integer

    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      Serial.println("Servo moved to angle: " + String(angle));
    } else {
      Serial.println("Invalid angle! Enter 0-180.");
    }
  }
}
