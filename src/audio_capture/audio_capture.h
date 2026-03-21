#pragma once

#include <Arduino.h>
#include "../system/shared_types.h"

void initAudioCapture();

bool updateSamplingStateFromButton(bool currentSamplingState);

void collectAudioFrame(AudioFrame& frame);