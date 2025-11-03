#include <Arduino.h>

// === Dual Motor Driver (TB6612FNG) + Command Interface ===
// Board: ESP32-WROOM-32UE
// Framework: Arduino (PlatformIO)

#define AIN1 12
#define AIN2 13
#define PWMA 14

#define BIN1 25
#define BIN2 26
#define PWMB 27

#define LED_STATUS 17

// PWM channels
#define CH_A 0
#define CH_B 1

// Blink timer  
unsigned long prevBlink = 0;
const unsigned long BLINK_INTERVAL = 1000; // ms

// Serial input buffer
String serialInput = "";

// === MOTOR CONTROL FUNCTION ===
void setMotor(char motor, int speed) {
  speed = constrain(speed, -255, 255);

  int in1, in2, pwmChannel;
  if (motor == 'A') { in1 = AIN1; in2 = AIN2; pwmChannel = CH_A; }
  else if (motor == 'B') { in1 = BIN1; in2 = BIN2; pwmChannel = CH_B; }
  else return;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

// === SETUP ===
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);

  ledcSetup(CH_A, 20000, 8);
  ledcSetup(CH_B, 20000, 8);
  ledcAttachPin(PWMA, CH_A);
  ledcAttachPin(PWMB, CH_B);

  Serial.println("\nTB6612FNG Dual Motor Control Ready!");
  Serial.println("Commands (press ENTER after typing):");
  Serial.println("  A f <0–255>   -> Motor A forward");
  Serial.println("  A b <0–255>   -> Motor A backward");
  Serial.println("  B f <0–255>   -> Motor B forward");
  Serial.println("  B b <0–255>   -> Motor B backward");
  Serial.println("  stop          -> Stop all motors\n");
}

// === LOOP ===
void loop() {
  // LED heartbeat (1 Hz)
  unsigned long now = millis();
  if (now - prevBlink >= BLINK_INTERVAL) {
    prevBlink = now;
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
  }

  // Handle serial commands (wait for Enter)
  if (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialInput.length() > 0) {
        // Process full line command
        Serial.println("> " + serialInput);

        // Tokenize
        char motor;
        char dir;
        int speed;

        if (serialInput.startsWith("stop")) {
          setMotor('A', 0);
          setMotor('B', 0);
          Serial.println("Motors stopped.");

        } else if (sscanf(serialInput.c_str(), "%c %c %d", &motor, &dir, &speed) == 3) {
          motor = toupper(motor);
          dir = tolower(dir);

          if (dir == 'f') setMotor(motor, speed);
          else if (dir == 'b') setMotor(motor, -speed);
          else Serial.println("Invalid direction!");

          Serial.printf("Motor %c set to %s %d\n", motor, dir == 'f' ? "forward" : "backward", abs(speed));
        } else {
          Serial.println("Invalid command format.");
        }

        serialInput = ""; // clear buffer
      }
    } else {
      serialInput += c; // build command string
    }
  }
}
