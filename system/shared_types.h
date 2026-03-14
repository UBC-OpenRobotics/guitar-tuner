#pragma once

#include <cstdint>
#include <vector>
#include "../config.h"

struct AudioFrame {
    int16_t samples[SAMPLE_COUNT]; // Array of audio samples 
    unsigned long timestampMs;      // Timestamp when sampling was completed
};

struct PitchResult {
    float frequencyHz; // The frequency
    float centsDeviation; // Deviation from the note
    char nearestNote[4]; // Name for the nearest note. NOTE: Can use size 3 (since we don't have sharps or flats), just did 4 to be safe though.
    unsigned long timestampMs; // Time of recording
};

struct MotorCommand {
    bool enabled;
    int direction = 0;
    int pwm = 0;
};

struct PIDController {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float integral = 0.0f;
    float previousError = 0.0f;

    float outputMin = -255.0f;
    float outputMax = 255.0f;

    unsigned long previousTimeMs = 0;
    bool firstRun = true;
};