#ifndef PINS_H
#define PINS_H

// ================== IR SENSOR ARRAYS ==================
#define IR1_PIN        32  // MUX output for Array 1
#define IR2_PIN        33  // MUX output for Array 2
#define MUX_S0         25
#define MUX_S1         26
#define MUX_S2         27

// ================== CAMERA MODULE (Pixy2) ==============
#define PIXY_SCK       18
#define PIXY_MOSI      23
#define PIXY_MISO      19
#define PIXY_CS        5

// ================== ULTRASONIC SENSOR ==================
#define ULTRASONIC_PIN 2   // Trigger + Echo (Grove ultrasonic)

// ================== MOTOR DRIVER (TB6612FNG) ===========
#define AIN1           12
#define AIN2           13
#define PWMA           14
#define STBY           -1  // Optional: define if you use STBY pin

// ================== MOTOR ENCODER =======================
#define ENCA           35
#define ENCB           34

// ================== SERVO MOTOR =========================
#define SERVO_PIN      16

// ================== IMU SENSOR (LSM6DS3) ===============
#define IMU_SCL        22
#define IMU_SDA        21

// ================== USER BUTTONS ========================
#define BTN_BOOT       0
#define BTN_RESET      255  // 'EN' pin is hardware reset (no GPIO control)

// ================== INDICATOR LEDs ======================
#define LED_RED        15
#define LED_BLUE       17

// ================== POWER MODULES =======================
// (No GPIO control — just for documentation)

#endif
