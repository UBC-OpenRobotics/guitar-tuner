#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "config.h"
#include "system/shared_types.h"

#include "audio_capture/audio_capture.h"
#include "pitch_detection/pitch_detector.h"
#include "control/pid_controller.h"
#include "control/motor_drive.h"

// Queues
QueueHandle_t g_audioQueue = nullptr;
QueueHandle_t g_pitchQueue = nullptr;
QueueHandle_t g_motorQueue = nullptr;

// Global modules/state
PitchDetector g_pitchDetector;
PIDController g_pid;

bool g_samplingActive = false;

// TODO: Example target frequency until configured from IO 
float targetFrequencyHz = 110.0f;

// Audio Tasks
void audioTask(void* pvParameters) {
    (void)pvParameters;

    AudioFrame frame;

    while (true) {

        // TODO: Place holder for the audio capture
        // g_samplingActive = updateSamplingStateFromButton(g_samplingActive);

        // if (!g_samplingActive) {
        //     vTaskDelay(pdMS_TO_TICKS(20));
        //     continue;
        // }

        // collectAudioFrame(frame);

        // Keep only the newest frame
        xQueueOverwrite(g_audioQueue, &frame);
    }
}

// Pitch Task
void pitchTask(void* pvParameters) {
    (void)pvParameters;

    AudioFrame frame;

    while (true) {
        if (xQueueReceive(g_audioQueue, &frame, portMAX_DELAY) == pdTRUE) {
            PitchResult pitch = g_pitchDetector.detectPitch(frame);

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
            float pidOutput = computePID(g_pid, targetFrequencyHz, pitch);
            MotorCommand cmd = buildMotorCommand(pidOutput);

            xQueueOverwrite(g_motorQueue, &cmd);
            
            Serial.println("PID Output: " + String(pidOutput));
            Serial.println("Motor Command sent: " + String(cmd.enabled) + " " + String(cmd.direction) + " " + String(cmd.pwm));
        }
    }
}

void motorTask(void* pvParameters) {
    (void)pvParameters;

    MotorCommand cmd;

    while (true) {
        if (xQueueReceive(g_motorQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            Serial.println("Motor Command received: " + String(cmd.enabled) + " " + String(cmd.direction) + " " + String(cmd.pwm));
        }
    }   
}

// Arduino setup/loop
void setup() {
    Serial.begin(115200);
    delay(500);

    // TODO: Place holder for the audio capture
    //initAudioCapture();

    initPIDController(
        g_pid,
        PID_KP,
        PID_KI,
        PID_KD,
        PID_OUTPUT_MIN,
        PID_OUTPUT_MAX
    );

    g_audioQueue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(AudioFrame));
    g_pitchQueue = xQueueCreate(PITCH_QUEUE_LENGTH, sizeof(PitchResult));
    g_motorQueue = xQueueCreate(MOTOR_QUEUE_LENGTH, sizeof(MotorCommand));

    if (g_audioQueue == nullptr || g_pitchQueue == nullptr || g_motorQueue == nullptr) {
        // lol
        while (true) {
            delay(1000);
            Serial.println("ERROR: failed to create one or more queues");
        }
    }

    xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        AUDIO_TASK_STACK,
        nullptr,
        AUDIO_TASK_PRIORITY,
        nullptr,
        AUDIO_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        pitchTask,
        "PitchTask",
        PITCH_TASK_STACK,
        nullptr,
        PITCH_TASK_PRIORITY,
        nullptr,
        PITCH_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        pidTask,
        "PIDTask",
        PID_TASK_STACK,
        nullptr,
        PID_TASK_PRIORITY,
        nullptr,
        PID_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        motorTask,
        "MotorTask",
        MOTOR_TASK_STACK,
        nullptr,
        MOTOR_TASK_PRIORITY,
        nullptr,
        MOTOR_TASK_CORE
    );
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
