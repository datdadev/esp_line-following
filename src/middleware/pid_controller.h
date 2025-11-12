#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

// PID Controller structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float min_output;   // Minimum output limit
    float max_output;   // Maximum output limit
    
    float integral;     // Accumulated integral error
    float last_error;   // Error from previous computation
    float last_output;  // Last computed output
} PIDController;

// Initialize the PID controller
PIDController pid_init(float kp, float ki, float kd, float min_output, float max_output);

// Compute the PID output based on setpoint and current value
float pid_compute(PIDController* pid, float setpoint, float current_value, float dt);

// Reset the PID controller (clears accumulated error)
void pid_reset(PIDController* pid);

// Set new PID parameters
void pid_setParameters(PIDController* pid, float kp, float ki, float kd);

// Set output limits
void pid_setOutputLimits(PIDController* pid, float min_output, float max_output);

// Get current PID parameters
void pid_getParameters(PIDController* pid, float* kp, float* ki, float* kd);

// Get last computed output
float pid_getLastOutput(PIDController* pid);

#endif // PID_CONTROLLER_H