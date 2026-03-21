#include "audio_capture.h"
#include "../config.h"

#include <math.h>

namespace {
    bool g_previousPressed = false;
    unsigned long g_pressStartMs = 0;
}

void initAudioCapture() {
    
    // 12-bit ADC
    analogReadResolution(12);

    // Expand measurable range toward 3.3V
    analogSetAttenuation(ADC_11db);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    g_previousPressed = (digitalRead(BUTTON_PIN) == LOW);
    g_pressStartMs = 0;
}

bool updateSamplingStateFromButton(bool currentSamplingState) {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);

    bool risingEdge = pressed && !g_previousPressed;
    bool fallingEdge = !pressed && g_previousPressed;

    if (risingEdge) {
        g_pressStartMs = millis();
    } 
    else if (fallingEdge) {
        unsigned long heldMs = millis() - g_pressStartMs;

        if (heldMs >= BUTTON_HOLD_MS) {
            currentSamplingState = !currentSamplingState;

            if (currentSamplingState) {
                Serial.println("\n=== SAMPLING STARTED ===");
            } else {
                Serial.println("\n=== SAMPLING STOPPED ===");
            }
        }
    }

    g_previousPressed = pressed;
    return currentSamplingState;
}

void collectAudioFrame(AudioFrame& frame) {
    unsigned long nextSampleTime = micros();

    for (size_t i = 0; i < SAMPLE_COUNT; i++) {
        while (micros() < nextSampleTime) {
            // wait until next sample instant
        }

        int raw = analogRead(MIC_PIN);
        frame.samples[i] = static_cast<int16_t>(raw);

        nextSampleTime += SAMPLE_PERIOD_US;
    }

    frame.timestampMs = millis();
}