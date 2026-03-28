#pragma once

#include <cstdint>

#include "../system/shared_types.h"

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
float detectFrequencyAutoCor(const int16_t* samples, int count, int sampleRate);
