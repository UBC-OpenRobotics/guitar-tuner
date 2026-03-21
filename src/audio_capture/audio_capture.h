#pragma once

#include "src/system/shared_types.h"

class AudioCapture {
public:
    void init();
    void collectSamples(AudioFrame& frame); 
private:
    void applyEMALowPassFilter(int16_t* samples, int count); 
    int calibrateDCOffset(const int16_t* samples, int count); 
};


