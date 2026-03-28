#pragma once

#include "../system/shared_types.h"

class AudioCapture {
public:
    /**
     * @brief Initialize audio capture setup
     */
    void init();

    /**
     * @brief Collects the samples
     * @param[in, out] frame Collects samples and stores them into the AudioFrame struct
     */
    void collectSamples(AudioFrame& frame); 
private:
    /**
     * @brief Applies the low pass filter to the set of samples
     * @param[in] samples Pointer to a set of samples
     * @param[in] count The number of samples
     */
    void applyEMALowPassFilter(int16_t* samples, int count);
    
    /**
     * @brief Calbraites the dc offset (the middle line)
     * @param[in] samples Pointer to a set of samples
     * @param[in] count The number of samples
     * @returns  The offset line
     */
    int calibrateDCOffset(const int16_t* samples, int count); 
};


