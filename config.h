#pragma once

// Audio Configuration
const int SAMPLE_RATE = 8000;
const int SAMPLE_COUNT = 1024;

// Guitar Frequency Bounds (Hz) - Low 82.41 (E2) to High 329.63 (E4)
const float MIN_FREQ = 60.0f;
const float MAX_FREQ = 400.0f;

//Autocorrelation Stability Configuration
const int REQUIRED_STABLE_READS = 3;
const float FREQ_TOLERANCE = 2.0f;
const long MIN_CORRELATION_THRESHOLD = 5000000;
