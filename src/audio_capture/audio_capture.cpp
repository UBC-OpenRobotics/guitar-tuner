#include "audio_capture.h"
#include "../config.h"
#include <Arduino.h>

/**
 * @brief Initialize audio capture setup
 */
void AudioCapture::init() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(MIC_PIN, INPUT);
}

// Might switch this in the future (don't need low pass)
/**
 * @brief Applies the low pass filter to the set of samples
 * @param[in] samples Pointer to a set of samples
 * @param[in] count The number of samples
 */
void AudioCapture::applyEMALowPassFilter(int16_t* samples, int count) {
    float alpha = 0.4; // The smoothing factor. Lower = smoother, but distorts the wave if too low.
      float smoothedValue = samples[0]; 
      
      for (int i = 1; i < SAMPLE_COUNT; i++) {
        // Calculate the moving average
        smoothedValue = (alpha * samples[i]) + ((1.0 - alpha) * smoothedValue);
        // Overwrite the raw sample with the smoothed sample
        samples[i] = (int16_t)smoothedValue; 
      }
}

/**
 * @brief Calbraites the dc offset (the middle line)
 * @param[in] samples Pointer to a set of samples
 * @param[in] count The number of samples
 * @returns  The offset line
 */
int AudioCapture::calibrateDCOffset(const int16_t* samples, int count) {
    long sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum += samples[i];
    }
    return sum / SAMPLE_COUNT;
}

/**
 * @brief Collects the samples
 * @param[in, out] frame Collects samples and stores them into the AudioFrame struct
 */
void AudioCapture::collectSamples(AudioFrame& frame) {
    const unsigned long SAMPLE_PERIOD_US = 1000000 / SAMPLE_RATE;
    unsigned long nextSampleTime = micros();

    // Collect samples
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        while (micros() < nextSampleTime) 
        frame.samples[i] = analogRead(MIC_PIN);
        nextSampleTime += SAMPLE_PERIOD_US;
    }
    
    // Record the timestmap
    frame.timestampMs = millis();
    
    // Record offset (middle line)
    int dcOffset = calibrateDCOffset(frame.samples, SAMPLE_COUNT); 
    
    // Apply low-pass filter
    applyEMALowPassFilter(frame.samples, SAMPLE_COUNT);
    
    // Apply offset to the currenet values
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        frame.samples[i] -= dcOffset;
    }
}



