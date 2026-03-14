#pragma once

#include <cstdint>

#include "system/shared_types.h"

// Pure function to detect frequency using autocorrelation
float detectFrequencyAutoCor(const int16_t* samples, int count, int sampleRate);
