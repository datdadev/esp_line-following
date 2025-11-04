#include <Arduino.h>
#include "pins.h"

// ====================== CONFIG ======================
constexpr float Ts = 0.01f;          // Sampling period [s]
constexpr float freq = 0.5f;         // Sine frequency [Hz]
constexpr int amplitude = 255;

// Encoder parameters
constexpr int PPR_motor = 11;        // pulses per motor shaft revolution
constexpr float gear_ratio = 10.0f;  // gear reduction
constexpr int quad_factor = 4;       // x4 decoding
constexpr float counts_per_rev = PPR_motor * gear_ratio * quad_factor; // 440 counts/rev

// ====================== TIMER ======================
hw_timer_t *timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ====================== SHARED VARIABLES ======================
volatile bool flag_sample = false;
volatile long encoderCount = 0;
volatile unsigned long sampleCount = 0;

// Quadrature encoder state variables
volatile int8_t lastEncoded = 0;

// ====================== INTERRUPTS ======================
void IRAM_ATTR onEncoder() {
  // Read both encoder pins
  int16_t MSB = digitalRead(ENCA);  // Most significant bit
  int16_t LSB = digitalRead(ENCB);  // Least significant bit
  
  // Combine the two pin states into a single value
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = (lastEncoded << 2) | encoded;
  
  // Increment or decrement based on the transition pattern
  // Clockwise/Forward: 00->01->11->10->00 (0->1->3->2->0)
  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
    encoderCount++;  // Forward direction
  }
  // Counter-clockwise/Reverse: 00->10->11->01->00 (0->2->3->1->0)
  else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) {
    encoderCount--;  // Reverse direction
  }
  
  // Store the current state for next interrupt
  lastEncoded = encoded;
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  flag_sample = true;
  sampleCount++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA), onEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), onEncoder, CHANGE);

  // Initialize the lastEncoded value with current state
  lastEncoded = (digitalRead(ENCA) << 1) | digitalRead(ENCB);

  // Timer setup → 100 Hz (10 ms)
  timer = timerBegin(0, 80, true);            // 80 MHz / 80 = 1 MHz → 1 µs/tick
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, Ts * 1e6, true);
  timerAlarmEnable(timer);

  Serial.println("=== Motor Control + Encoder RPM Test (10ms) ===");
}

// ====================== LOOP ======================
void loop() {
  static long prevCount = 0;

  if (flag_sample) {
    portENTER_CRITICAL(&timerMux);
    flag_sample = false;
    unsigned long k = sampleCount;
    portEXIT_CRITICAL(&timerMux);

    // Heartbeat LED (0.5 Hz blink)
    digitalWrite(LED_BLUE, (k / 50) % 2);

    // ==== Generate PWM ====
    float theta = 2 * PI * freq * (k * Ts);
    int pwmValue = (int)((sin(theta) * 0.5f + 0.5f) * amplitude);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwmValue);

    // ==== Compute RPM ====
    long currentCount;
    portENTER_CRITICAL(&timerMux);
    currentCount = encoderCount;
    portEXIT_CRITICAL(&timerMux);

    long deltaCount = currentCount - prevCount;
    prevCount = currentCount;

    float rev_per_sample = deltaCount / counts_per_rev;
    float rpm = (rev_per_sample / Ts) * 60.0f;

    // ==== Log ====
    Serial.printf("%lu, %.3f, %d, %ld, %.2f\n",
                  k, k * Ts, pwmValue, currentCount, rpm);
  }
}
