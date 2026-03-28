#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "src/config.h"
#include "src/system/shared_types.h"

#include "src/audio_capture/audio_capture.h"
#include "src/pitch_detection/pitch_detector.h"
#include "src/control/pid_controller.h"
#include "src/control/motor_drive.h"

// Define these missing constants if not in config.h yet
// They were used in guitar-tuner.ino but might not be defined if I missed config updates
// (Removed local defines to use config.h values)

// Queues
QueueHandle_t g_audioQueue = nullptr;
QueueHandle_t g_pitchQueue = nullptr;
QueueHandle_t g_motorQueue = nullptr;

// Global modules/state
AudioCapture g_audioCapture;
PitchDetector g_pitchDetector;
PIDController g_pid;

bool g_samplingActive = false;
float targetFrequencyHz = 110.0f; // Example target

// Forward declarations
void audioTask(void* pvParameters);
void pitchTask(void* pvParameters);
void pidTask(void* pvParameters);
void motorTask(void* pvParameters);

// Audio Task
void audioTask(void* pvParameters) {
    (void)pvParameters;
    AudioFrame frame;
    
    // Initialize Audio Capture
    g_audioCapture.init();

    // Button State
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    int previousButtonState = digitalRead(BUTTON_PIN);
    unsigned long startPress = millis();
    bool risingEdge, fallingEdge;

    // Use a small loop delay tick for handling the button + capturing
    // Loop control
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Polling frequency

    while (true) {
        // Handle Button Logic (Toggle Sampling)
        int currentButtonState = digitalRead(BUTTON_PIN);
        risingEdge = !currentButtonState && previousButtonState;
        fallingEdge = currentButtonState && !previousButtonState;
        
        if (risingEdge) {
            startPress = millis();
        } else if (fallingEdge) {
            unsigned long duration = millis() - startPress;
            if (duration > 1000) {
                 // Long press detected - toggle sampling
                g_samplingActive = !g_samplingActive;
                if (g_samplingActive) {
                    Serial.println("=== SAMPLING STARTED ===");
                } else {
                    Serial.println("=== SAMPLING STOPPED ===");
                }
            }
        }
        previousButtonState = currentButtonState;

        if (g_samplingActive) {
             g_audioCapture.collectSamples(frame);
             // Use xQueueOverwrite for 1-item queues (config.h)
             xQueueOverwrite(g_audioQueue, &frame);
             // Force a yield to feed the watchdog
             vTaskDelay(pdMS_TO_TICKS(10)); 
        } else {
             // If not sampling, delay normally
             vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Pitch Task
void pitchTask(void* pvParameters) {
    (void)pvParameters;
    AudioFrame frame;
    while (true) {
        if (xQueueReceive(g_audioQueue, &frame, portMAX_DELAY) == pdTRUE) {
            PitchResult pitch = g_pitchDetector.detectPitch(frame);
            
            if (pitch.frequencyHz > 0) 
            // Print debugging info onto Serial monitor
            Serial.printf("Freq: %.2f Hz | Note: %s | Deviation: %.2f cents\n", 
                          pitch.frequencyHz, pitch.nearestNote, pitch.centsDeviation);

            xQueueOverwrite(g_pitchQueue, &pitch);
        }
    }
}

// PID Task
void pidTask(void* pvParameters) {
    (void)pvParameters;
    PitchResult pitch;
    while (true) {
        if (xQueueReceive(g_pitchQueue, &pitch, portMAX_DELAY) == pdTRUE) {
            // Assume method exists or adapt
            // float error = targetFrequencyHz - pitch.frequencyHz;
            // updatePID(g_pid, error); 
            
            MotorCommand cmd;
            cmd.enabled = true;
            cmd.pwm = 0; // Placeholder
            cmd.direction = 0;
            
            xQueueOverwrite(g_motorQueue, &cmd);
            
            // Serial.println("Pitch processed.");
        }
    }
}

// Motor Task
void motorTask(void* pvParameters) {
    (void)pvParameters;
    MotorCommand cmd;
    while (true) {
        if (xQueueReceive(g_motorQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            // motor.setSpeed(cmd.speed);
            // Serial.println("Motor command received.");
        }
    }
}

// Standard Arduino entry points
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Guitar Tuner Initializing...");

    // Initialize PID (assuming function exists)
    // initPIDController(g_pid, PID_KP, PID_KI, PID_KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    g_audioQueue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(AudioFrame));
    g_pitchQueue = xQueueCreate(PITCH_QUEUE_LENGTH, sizeof(PitchResult));
    g_motorQueue = xQueueCreate(MOTOR_QUEUE_LENGTH, sizeof(MotorCommand));

    if (!g_audioQueue || !g_pitchQueue || !g_motorQueue) {
        Serial.println("Error creating queues!");
        while(1) delay(1000);
    }

    xTaskCreatePinnedToCore(audioTask, "AudioTask", AUDIO_TASK_STACK, nullptr, AUDIO_TASK_PRIORITY, nullptr, AUDIO_TASK_CORE);
    xTaskCreatePinnedToCore(pitchTask, "PitchTask", PITCH_TASK_STACK, nullptr, PITCH_TASK_PRIORITY, nullptr, PITCH_TASK_CORE);
    xTaskCreatePinnedToCore(pidTask, "PIDTask", PID_TASK_STACK, nullptr, PID_TASK_PRIORITY, nullptr, PID_TASK_CORE);
    xTaskCreatePinnedToCore(motorTask, "MotorTask", MOTOR_TASK_STACK, nullptr, MOTOR_TASK_PRIORITY, nullptr, MOTOR_TASK_CORE);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

