#ifndef ERROR_CODES_H
#define ERROR_CODES_H

// ===================== DEBUG FLAG =====================
// Uncomment the line below to enable debug output
// #define DEBUG_ENABLED

// ===================== ERROR CODES =====================
// Success codes (non-negative values)
#define ERROR_SUCCESS                   0    // Operation completed successfully

// General error codes (negative values)
#define ERROR_GENERAL_FAILURE          -1    // General failure - catch-all error
#define ERROR_INVALID_PARAMETER        -2    // Invalid parameter passed to function
#define ERROR_OUT_OF_MEMORY            -3    // Memory allocation failed
#define ERROR_TIMEOUT                  -4    // Operation timed out
#define ERROR_NOT_INITIALIZED          -5    // Component/module not initialized
#define ERROR_ALREADY_INITIALIZED      -6    // Component/module already initialized
#define ERROR_RESOURCE_BUSY            -7    // Resource is currently busy
#define ERROR_ACCESS_DENIED            -8    // Access to resource denied
#define ERROR_NOT_SUPPORTED            -9    // Operation not supported
#define ERROR_NOT_FOUND                -10   // Requested resource not found
#define ERROR_IO_FAILURE               -11   // Input/output operation failed
#define ERROR_COMMUNICATION            -12   // Communication with device failed
#define ERROR_BUFFER_OVERFLOW          -13   // Buffer overflow occurred
#define ERROR_INVALID_STATE            -14   // Component is in invalid state for operation

// Sensor-specific error codes
#define ERROR_SENSOR_READ_FAILED       -100  // Sensor reading failed
#define ERROR_SENSOR_NOT_CONNECTED     -101  // Sensor not properly connected
#define ERROR_SENSOR_CALIBRATION       -102  // Sensor calibration failed
#define ERROR_ULTRASONIC_TIMEOUT       -103  // Ultrasonic sensor reading timeout

// Motor-specific error codes
#define ERROR_MOTOR_INIT_FAILED        -200  // Motor initialization failed
#define ERROR_MOTOR_DRIVER_FAULT       -201  // Motor driver fault detected
#define ERROR_ENCODER_FAULT            -202  // Encoder fault detected
#define ERROR_MOTOR_OVER_CURRENT       -203  // Motor over current condition

// Servo-specific error codes
#define ERROR_SERVO_INIT_FAILED        -50  // Servo initialization failed
#define ERROR_SERVO_OUT_OF_RANGE       -51  // Attempted to set servo to invalid angle
#define ERROR_SERVO_FAULT              -52  // Servo fault detected

// PID controller error codes
#define ERROR_PID_INVALID_CONFIG       -60  // Invalid PID configuration parameters
#define ERROR_PID_CALCULATION          -61  // PID calculation error

// State machine error codes
#define ERROR_STATE_INVALID_TRANSITION -70  // Invalid state transition attempted
#define ERROR_STATE_UNEXPECTED         -71  // Unexpected state detected

// ===================== DEBUG MACRO =====================
#ifdef DEBUG_ENABLED
    #define DEBUG_PRINT(x) Serial.println(x)
    #define DEBUG_PRINT2(x, y) Serial.print(x); Serial.println(y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINT2(x, y)
#endif

#endif // ERROR_CODES_H