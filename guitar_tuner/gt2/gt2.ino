/*
 * Frequency Detection with MAX9814 and ESP32
 * 
 * Pin Connections:
 * MAX9814 VDD -> 3V3
 * MAX9814 GND -> GND
 * MAX9814 OUT -> GPIO 35
 * BUTTON -> GPIO 14
 * 
 * Button starts/stops periodic frequency sampling.
 * Detected frequency is printed to Serial console.
 */

// Pin definitions (same as gt.ino)
const int micPin = 35;
const int buttonPin = 14;

// Sampling configuration
const int SAMPLE_RATE = 8000;              // 8 kHz sample rate (good for guitar: 80-400 Hz)
const int SAMPLE_COUNT = 1024;             // Number of samples per measurement
const unsigned long SAMPLE_PERIOD_US = 1000000 / SAMPLE_RATE;  // Microseconds between samples
const unsigned long PRINT_INTERVAL_MS = 500;  // Print frequency every 500ms

// Sample buffer
int16_t samples[SAMPLE_COUNT];

// State variables
bool samplingActive = false;
unsigned int currentButtonState;
unsigned int previousButtonState;
unsigned long startPress = 0;
unsigned long lastPrintTime = 0;

// DC offset (will be calibrated)
int dcOffset = 2048;  // Default to mid-scale for 12-bit ADC

void setup() {
  // Higher baud rate for faster serial output
  Serial.begin(115200);
  
  // 12 bit ADC (0-4095)
  analogReadResolution(12);
  
  // Attenuation from 1.1V --> 3.3V
  analogSetAttenuation(ADC_11db);
  
  // Configure button pin as input with pull-up
  pinMode(buttonPin, INPUT_PULLUP);
  
  previousButtonState = digitalRead(buttonPin);
  
  Serial.println("Frequency Detector Ready");
  Serial.println("Press and hold button (>1s) to start/stop sampling");
}

void loop() {
  // Handle button input
  handleButton();
  
  // If sampling is active, collect samples and detect frequency
  if (samplingActive) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
      // Collect samples
      collectSamples();
      
      // Calibrate DC offset
      calibrateDCOffset();
      
      // Detect frequency using zero-crossing
      float frequency = detectFrequency();
      
      // Print results
      if (frequency > 0) {
        Serial.print("Detected Frequency: ");
        Serial.print(frequency, 1);
        Serial.println(" Hz");
        
        // Print corresponding note (for reference)
        printNearestNote(frequency);
      } else {
        Serial.println("No frequency detected (signal too quiet or no periodicity)");
      }
      
      lastPrintTime = currentTime;
    }
  }
}

void handleButton() {
  currentButtonState = digitalRead(buttonPin);
  
  bool risingEdge = !currentButtonState && previousButtonState;
  bool fallingEdge = currentButtonState && !previousButtonState;
  
  if (risingEdge) {
    startPress = millis();
  }
  else if (fallingEdge && (millis() - startPress > 1000)) {
    // Long press detected - toggle sampling
    samplingActive = !samplingActive;
    
    if (samplingActive) {
      Serial.println("\n=== SAMPLING STARTED ===");
      lastPrintTime = 0;  // Force immediate measurement
    } else {
      Serial.println("\n=== SAMPLING STOPPED ===");
    }
  }
  
  previousButtonState = currentButtonState;
}

void collectSamples() {
  unsigned long nextSampleTime = micros();
  
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    // Wait for the next sample time
    while (micros() < nextSampleTime) {
      // Busy wait for precise timing
    }
    
    samples[i] = analogRead(micPin);
    nextSampleTime += SAMPLE_PERIOD_US;
  }
}

void calibrateDCOffset() {
  // Calculate average (DC offset) of the samples
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += samples[i];
  }
  dcOffset = sum / SAMPLE_COUNT;
}

float detectFrequency() {
  // Zero-crossing detection with hysteresis
  const int HYSTERESIS = 50;  // Threshold to avoid noise triggering crossings
  
  int crossingCount = 0;
  int firstCrossingIndex = -1;
  int lastCrossingIndex = -1;
  
  // Find zero crossings (from below to above only, for consistency)
  for (int i = 1; i < SAMPLE_COUNT; i++) {
    int value = samples[i] - dcOffset;
    int prevValue = samples[i - 1] - dcOffset;
    
    // Detect rising edge crossing
    if (prevValue < -HYSTERESIS && value > HYSTERESIS) {
      if (firstCrossingIndex == -1) {
        firstCrossingIndex = i;
      }
      lastCrossingIndex = i;
      crossingCount++;
    }
  }
  
  // Need at least 2 crossings to calculate frequency
  if (crossingCount < 2) {
    return 0;
  }
  
  // Calculate frequency from number of crossings and time span
  int sampleSpan = lastCrossingIndex - firstCrossingIndex;
  float timeSpanSeconds = (float)sampleSpan / SAMPLE_RATE;
  float periods = crossingCount - 1;  // Number of complete periods
  
  float frequency = periods / timeSpanSeconds;
  
  // Sanity check: guitar frequencies are typically 80-1200 Hz
  if (frequency < 60 || frequency > 1500) {
    return 0;
  }
  
  return frequency;
}

void printNearestNote(float frequency) {
  // Standard guitar tuning frequencies (E2 A2 D3 G3 B3 E4)
  const char* noteNames[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
  const float noteFreqs[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63};
  const int noteCount = 6;
  
  // Find closest note
  int closestIdx = 0;
  float minDiff = abs(frequency - noteFreqs[0]);
  
  for (int i = 1; i < noteCount; i++) {
    float diff = abs(frequency - noteFreqs[i]);
    if (diff < minDiff) {
      minDiff = diff;
      closestIdx = i;
    }
  }
  
  // Calculate cents offset
  float cents = 1200 * log2(frequency / noteFreqs[closestIdx]);
  
  Serial.print("  Nearest note: ");
  Serial.print(noteNames[closestIdx]);
  Serial.print(" (");
  Serial.print(noteFreqs[closestIdx], 2);
  Serial.print(" Hz), ");
  
  if (cents > 0) {
    Serial.print("+");
  }
  Serial.print(cents, 1);
  Serial.println(" cents");
}
