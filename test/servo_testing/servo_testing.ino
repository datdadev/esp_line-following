#include <Arduino.h>

// Pin definition
const int SERVO_PIN = 16;

void setup() {
  Serial.begin(115200);
  pinMode(SERVO_PIN, OUTPUT);
  Serial.println("ESP32 Servo Test on GPIO16");
  Serial.println("Enter an angle between 0 and 180:");
}

// FIXED: Continuous servo control
void setServoAngle(uint8_t angle) {
    angle = constrain(angle, 0, 180);
    uint16_t pulseWidth = map(angle, 0, 180, 500, 2400);
    
    // Generate the PWM signal for one complete cycle (20ms)
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(1000 - pulseWidth); // Complete the 20ms period
}

void loop() {
    // Continuously send servo signals at 50Hz
    static uint32_t lastServoTime = 0;
    static int currentAngle = 90; // Default angle
    
    // Update servo position every 20ms (50Hz)
    if (micros() - lastServoTime >= 1000) {
        setServoAngle(currentAngle);
        lastServoTime = micros();
    }
    
    // Check for serial input
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        int angle = input.toInt();
        
        if (angle >= 0 && angle <= 180) {
            currentAngle = angle;
            Serial.println("Servo moving to angle: " + String(angle));
        } else {
            Serial.println("Invalid angle! Enter 0-180.");
        }
    }
}



// #include <Arduino.h>
// #include <ESP32Servo.h>

// // Pin definition
// const int servoPin = 16;
// Servo myServo;

// void setup() {
//   Serial.begin(115200);
//   myServo.setPeriodHertz(50);        // Standard 50 Hz for servo
//   myServo.attach(servoPin, 500, 2500); // Min 500us, Max 2500us pulse width
//   Serial.println("ESP32 Servo Test on GPIO16");
//   Serial.println("Enter an angle between 0 and 180:");
// }

// void loop() {
//   // Check if user typed something in Serial
//   if (Serial.available() > 0) {
//     String input = Serial.readStringUntil('\n');
//     input.trim();
    
//     int angle = input.toInt(); // Convert input to integer

//     if (angle >= 0 && angle <= 180) {
//       myServo.write(angle);
//       Serial.println("Servo moved to angle: " + String(angle));
//     } else {
//       Serial.println("Invalid angle! Enter 0-180.");
//     }
//   }
// }
