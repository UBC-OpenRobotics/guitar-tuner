#include <iostream>
#include "pitch_detection/pitch_algorithm.h"

#ifndef ARDUINO
using namespace std;

int main() {
    // Testing
    int16_t samples[1024] = {0}; 
    float frequency = detectFrequencyAutoCor(samples, 1024, 44100);
    cout << "Detected Frequency: " << frequency << endl;
    return 0;
}
#endif