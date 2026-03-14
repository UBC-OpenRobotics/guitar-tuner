#include "pid_controller.h"
#include "config.h"

#include <cmath>

static float clampValue(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void initPIDController(PIDController& pid,
                       float kp,
                       float ki,
                       float kd,
                       float outputMin,
                       float outputMax) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;

    pid.integral = 0.0f;
    pid.previousError = 0.0f;

    pid.outputMin = outputMin;
    pid.outputMax = outputMax;

    pid.previousTimeMs = 0;
    pid.firstRun = true;
}

void resetPIDController(PIDController& pid) {
    pid.integral = 0.0f;
    pid.previousError = 0.0f;
    pid.previousTimeMs = 0;
    pid.firstRun = true;
}

float computePID(PIDController& pid,
                 float targetFrequencyHz,
                 const PitchResult& pitch) {
    
    // If we want to add validation.
    // // if (!pitch.valid || pitch.frequencyHz <= 0.0f) {
    //     resetPIDController(pid);
    //     return 0.0f;
    // }

    float error = targetFrequencyHz - pitch.frequencyHz;

    // If it's the first run, set the previous time and error
    if (pid.firstRun) {
        pid.previousTimeMs = pitch.timestampMs;
        pid.previousError = error;
        pid.firstRun = false;
    }

    // Calculate the time difference, if 0, set to 0.001f (for example for the first run)
    float dt = static_cast<float>(pitch.timestampMs - pid.previousTimeMs) / 1000.0f;
    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    // Add to the integral
    pid.integral += error * dt;

    // Clamp the integral, to prevent windup
    const float integralClamp = 100.0f;
    pid.integral = clampValue(pid.integral, -integralClamp, integralClamp);

    // Calculate the derivative from the previous value
    float derivative = (error - pid.previousError) / dt;

    // Sum the three terms
    float output =
        pid.kp * error +
        pid.ki * pid.integral +
        pid.kd * derivative;

    // Clamp the output to the PWM range
    output = clampValue(output, pid.outputMin, pid.outputMax);

    // Update the previous params for the next iteration
    pid.previousError = error;
    pid.previousTimeMs = pitch.timestampMs;

    return output;
}

MotorCommand buildMotorCommand(float pidOutput) {
    MotorCommand cmd{};

    float magnitude = std::fabs(pidOutput);

    // If magnitude is less than motor, don't send command
    if (magnitude < MOTOR_OUTPUT_DEADBAND) {
        cmd.enabled = false;
        cmd.direction = 0;
        cmd.pwm = 0;
        return cmd;
    }

    // Set command 
    cmd.enabled = true;
    cmd.direction = (pidOutput > 0.0f) ? +1 : -1;

    // Round to nearest integer
    cmd.pwm = static_cast<int>(magnitude + 0.5f);
    return cmd;
}
