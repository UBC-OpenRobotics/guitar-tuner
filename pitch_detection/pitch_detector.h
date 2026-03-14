#pragma once

#include "../system/shared_types.h"

class PitchDetector {
public:
    PitchResult detectPitch(const AudioFrame& frame);

private:
    float lastFrequency = 0.0f;
    int stableCount = 0;
    
    void updateNearestNote(PitchResult& result, float frequency);
};
