#include "src/config.h"
#include "src/audio_capture/audio_capture.h"
#include "src/pitch_detection/pitch_detector.h"
#include "src/control/pid_controller.h"
#include "src/control/motor_drive.h"

// Instantiate global objects
// AudioCapture audio; (Example)
// PitchDetector pitchDetector; (Example)
// PIDController pid; (Example)
// MotorDrive motor; (Example)

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready
  
  Serial.println("Guitar Tuner Initializing...");
  
  // Initialize hardware
  // audio.init();
  // motor.init();
}

void loop() {
  // Main loop logic
  
  // 1. Capture Audio
  // float* buffer = audio.readBuffer();
  
  // 2. Detect Pitch
  // float frequency = pitchDetector.detect(buffer, SAMPLE_COUNT, SAMPLE_RATE);
  
  // 3. Control Motor
  // float error = TARGET_FREQ - frequency;
  // float output = pid.compute(error);
  // motor.drive(output);
  
  // delay(100);
}
