#include "pid_controller.h"
#include <algorithm>

PIDController pid_init(float kp, float ki, float kd, float min_output, float max_output) {
    PIDController pid;
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.min_output = min_output;
    pid.max_output = max_output;
    pid.integral = 0.0;
    pid.last_error = 0.0;
    pid.last_output = 0.0;
    return pid;
}

float pid_compute(PIDController* pid, float setpoint, float current_value, float dt) {
    if (!pid || dt <= 0) return 0.0;  // Safety check
    
    float error = setpoint - current_value;
    
    // Update integral with anti-windup
    pid->integral += error * dt;
    
    // Calculate derivative
    float derivative = (error - pid->last_error) / dt;
    
    // Compute PID output
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // Apply output limits
    output = std::max(pid->min_output, std::min(pid->max_output, output));
    
    // Store values for next iteration
    pid->last_error = error;
    pid->last_output = output;
    
    return output;
}

void pid_reset(PIDController* pid) {
    if (!pid) return;
    
    pid->integral = 0.0;
    pid->last_error = 0.0;
    pid->last_output = 0.0;
}

void pid_setParameters(PIDController* pid, float kp, float ki, float kd) {
    if (!pid) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_setOutputLimits(PIDController* pid, float min_output, float max_output) {
    if (!pid) return;
    
    pid->min_output = min_output;
    pid->max_output = max_output;
}

void pid_getParameters(PIDController* pid, float* kp, float* ki, float* kd) {
    if (!pid) return;
    
    if (kp) *kp = pid->kp;
    if (ki) *ki = pid->ki;
    if (kd) *kd = pid->kd;
}

float pid_getLastOutput(PIDController* pid) {
    if (!pid) return 0.0;
    
    return pid->last_output;
}