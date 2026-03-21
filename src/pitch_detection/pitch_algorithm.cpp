#include "pitch_detection/pitch_algorithm.h"
#include "config.h"

#include <cmath>
#include <cstdlib>

/**
 * @brief Detects the fundamental frequency of an audio signal using autocorrelation.
 * 
 * The autocorrelation algorithm.
 * Removes DC offset, calculates autocorrelation for lags corresponding
 * to the expected guitar frequency range (determined by MIN_FREQ and MAX_FREQ), 
 * and identifies the lag with the highest correlation/best similarities. 
 * If the correlation exceeds a noise threshold, the frequency is returned.
 * 
 * 
 * @param[in] samples The array of raw 16-bit audio samples.
 * @param[in] count The number of samples in the array.
 * @param[in] sampleRate The sampling rate in Hz.
 * @returns The detected frequency in Hz, otherwise 0.0f if no valid pitch is found.
 */
float detectFrequencyAutoCor(const int16_t* samples, int count, int sampleRate) {
    // Find DC Offset (the middle line/perfect silence)
    long sum = 0;
    for (int i = 0; i < count; i++) {
        sum += samples[i];
    }
    int dcOffset = sum / count;

    // Convert Hz to lags
    // The period is described as T = 1/f, but we only need to search for periods that respond to the guitar frequencies. 
    // So we make the numerator the sample rate.
    int minLag = sampleRate / MAX_FREQ;
    int maxLag = sampleRate / MIN_FREQ;

    // Initialize variables
    int64_t maxCorrelation = 0;
    int bestLag = -1;

    // Iterate through the possible lags (within one period)
    for (int lag = minLag; lag <= maxLag; lag++) {
        int64_t correlation = 0;

        // Iterate through the different waves to find the best match
        for (int i = 0; i < count - lag; i++) {
            int32_t sample1 = samples[i] - dcOffset;
            int32_t sample2 = samples[i + lag] - dcOffset;
            correlation += (int64_t)sample1 * sample2; // Multiplied since best matches will be peak * peak or valley * valley
        }

        // If this is the best correlation we've found
        if (correlation > maxCorrelation) {
            maxCorrelation = correlation;
            bestLag = lag;
        }
    }

    // If there was a wave and it's correlation threshold is met (the noise was loud enough), output a frequency
    if (bestLag > 0 && maxCorrelation > MIN_CORRELATION_THRESHOLD) {
            return (float)sampleRate / bestLag;
    }

    return 0.0f;
}
