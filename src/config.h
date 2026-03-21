#pragma once

#include <Arduino.h>

// Pins
constexpr int MIC_PIN = 35;
constexpr int BUTTON_PIN = 14;

// Sampling
constexpr int SAMPLE_RATE = 8000;
constexpr int SAMPLE_COUNT = 1024;

// Printing / timing (remove for now)
constexpr unsigned long BUTTON_HOLD_MS = 1000;
constexpr unsigned long NO_PITCH_PRINT_INTERVAL_MS = 500;

// Guitar Frequency Bounds
constexpr float MIN_FREQ = 60.0f;
constexpr float MAX_FREQ = 400.0f;

// Autocorrelation Stability Configuration
constexpr int REQUIRED_STABLE_READS = 3;
constexpr float FREQ_TOLERANCE = 2.0f;
constexpr long MIN_CORRELATION_THRESHOLD = 5000000;

// PID configuration (untuned for now)
constexpr float PID_KP = 10.0f;
constexpr float PID_KI = 4.0f;
constexpr float PID_KD = 8.0f;

// Clamp the PID output to a PWM-like range
constexpr float PID_OUTPUT_MIN = -255.0f;
constexpr float PID_OUTPUT_MAX = 255.0f;

// PID Output Deadband
constexpr float MOTOR_OUTPUT_DEADBAND = 8.0f;

// Task stack sizes
constexpr uint32_t AUDIO_TASK_STACK = 4096;
constexpr uint32_t PROCESS_TASK_STACK = 8192;

// Task priorities (lower = higher priority)
constexpr int AUDIO_TASK_PRIORITY = 2;
constexpr int PITCH_TASK_PRIORITY = 2;
constexpr int PID_TASK_PRIORITY = 2;
constexpr int MOTOR_TASK_PRIORITY = 1;

// Task cores (ESP32 dual-core: 0 or 1)
constexpr int AUDIO_TASK_CORE = 0;
constexpr int PITCH_TASK_CORE = 0;
constexpr int PID_TASK_CORE = 1;
constexpr int MOTOR_TASK_CORE = 1;

// Queue lengths
constexpr uint32_t AUDIO_QUEUE_LENGTH = 1;
constexpr uint32_t PITCH_QUEUE_LENGTH = 1;
constexpr uint32_t MOTOR_QUEUE_LENGTH = 1;