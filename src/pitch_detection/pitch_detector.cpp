#include "pitch_detection/pitch_detector.h"
#include "pitch_detection/pitch_algorithm.h"

#include <cmath> // For abs
#include <cstring> // For strncpy
#include <algorithm> // For min 

using namespace std; // For Daylen's brain to work with C++ syntax

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
PitchResult PitchDetector::detectPitch(const AudioFrame& frame) {
    PitchResult result; // The pitch result
    result.timestampMs = frame.timestampMs; // The timeframe (should come from audio_capture I think)
    result.frequencyHz = 0.0f; // The frequency
    result.centsDeviation = 0.0f; // The cents/error
    result.nearestNote[0] = '\0'; // Set a dummy null terminator in the nearest note so the string isn't empty/full of garbage

    // Get frequency result from pitch_algorithm.cpp
    float frequency = detectFrequencyAutoCor(frame.samples, SAMPLE_COUNT, SAMPLE_RATE);

    // Stability Filter Logic
    // Only accept frequencies within the expected guitar range (meant to void noise)
    if (frequency >= MIN_FREQ && frequency <= MAX_FREQ) {
        
        // If the reading is close to the previous one
        if (abs(frequency - lastFrequency) <= FREQ_TOLERANCE) {
            stableCount++;
        } else {
            stableCount = 1; // Reset if it jumped too much (there was noise)
        }
        
        // Update the last frequency to the current (similar to edge detection)
        lastFrequency = frequency;
        
        // Only consider it a defined pitch if it's stable
        if (stableCount >= REQUIRED_STABLE_READS) {
            result.frequencyHz = frequency;
            updateNearestNote(result, frequency);
        }
    } else {
        // Signal is out of range and/or it picked up unwanted noise
        stableCount = 0;
        lastFrequency = 0.0f;
    }
    
    return result;
}

/**
 * @brief Updates the PitchResult with the nearest musical note and cents deviation.
 * 
 * The identified frequency is compared against standard guitar tuning frequencies
 * (E2 to E4). The nearest note is selected, and the deviation/error in cents is calculated.
 * 
 * @param[out] result The PitchResult structure to update.
 * @param frequency The detected frequency in Hz.
 */
void PitchDetector::updateNearestNote(PitchResult& result, float frequency) {
    const char* noteNames[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
    const float noteFreqs[] = {82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f};
    const int noteCount = sizeof(noteNames) / sizeof(noteNames[0]);
  
    // Initialize variables
    int closestIdx = 0;
    float minDiff = abs(frequency - noteFreqs[0]);
  
    // Calculate the nearest note and how far it is from that nearest note's frequency
    for (int i = 1; i < noteCount; i++) {
        float diff = abs(frequency - noteFreqs[i]);
        if (diff < minDiff) {
            minDiff = diff;
            closestIdx = i;
        }
    }
  
    // Calculate deviation in cents
    // cents = 1200 * log2(f_measured / f_target)
    // 1200 = 12 semitones * 1 semitone with 100 cents of accuracy
    // Pitch is logarithmic, 110 Hz, 220 Hz, 440 Hz, etc 
    float cents = 1200.0f * log2(frequency / noteFreqs[closestIdx]);
    result.centsDeviation = cents;
    
    // Copy note name
    strncpy(result.nearestNote, noteNames[closestIdx], 3);
    result.nearestNote[3] = '\0';
}
