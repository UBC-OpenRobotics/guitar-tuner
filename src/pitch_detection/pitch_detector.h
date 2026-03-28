#pragma once

#include "../system/shared_types.h"

class PitchDetector {
public:
    /**
     * @brief Processes an audio frame to detect a stable pitch.
     * 
     * Autocorrelation algorithm is used to find a raw frequency. 
     * It ensures that the frequency is a valid note (determined by MIN_FREQ and MAX_FREQ). 
     * It then applies a stability filter (determined by FREQ_TOLERANCE) to ensure the frequency readings are consistent.
     * It then requires a specific number of consecutive consistent readings(defined by REQUIRED_STABLE_READS) 
     * before accepting a new pitch.
     * 
     * @param[out] frame The audio frame containing samples and timestamp.
     * @returns A PitchResult structure containing frequency, note name, and deviation.
     */
    PitchResult detectPitch(const AudioFrame& frame);

private:
    // Stores last frequency for comparison
    float lastFrequency = 0.0f;

    // The number of stable readings within the valid range
    int stableCount = 0;
    
    /**
     * @brief Updates the PitchResult with the nearest musical note and cents deviation.
     * 
     * The identified frequency is compared against standard guitar tuning frequencies
     * (E2 to E4). The nearest note is selected, and the deviation/error in cents is calculated.
     * 
     * @param[out] result The PitchResult structure to update.
     * @param frequency The detected frequency in Hz.
     */
    void updateNearestNote(PitchResult& result, float frequency);
};
