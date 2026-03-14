/*
 * Autocorrelation Pitch Detection with Stability Filter
 * * Pin Connections:
 * MAX9814 VDD -> 3V3
 * MAX9814 GND -> GND
 * MAX9814 OUT -> GPIO 35
 * BUTTON -> GPIO 14
 */

//---------------PARAMETERS---------------//
const int micPin = 35;
const int buttonPin = 14;

const int SAMPLE_RATE = 8000;
const int SAMPLE_COUNT = 1024;
const unsigned long SAMPLE_PERIOD_US = 1000000 / SAMPLE_RATE;

// Shortened interval! It will now sample as fast as the 128ms buffer allows.
const unsigned long PRINT_INTERVAL_MS = 10; 

// Guitar frequency bounds
const float MIN_FREQ = 60.0;
const float MAX_FREQ = 400.0;

// --- STABILITY FILTER PARAMETERS ---
const int REQUIRED_STABLE_READS = 3;  // How many times it must hear the SAME note
const float FREQ_TOLERANCE = 2.0;     // Hz difference allowed between reads
//----------------------------------------//

int16_t samples[SAMPLE_COUNT];

bool samplingActive = false;
unsigned int currentButtonState;
unsigned int previousButtonState;
unsigned long startPress = 0;
unsigned long lastPrintTime = 0;

// Stability tracking variables
float lastFrequency = 0.0;
int stableCount = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  pinMode(buttonPin, INPUT_PULLUP);
  previousButtonState = digitalRead(buttonPin);
}

void loop() {
  handleButton();
  
  if (samplingActive) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
      collectSamples();
      
      float frequency = detectFrequencyAutoCor();
      
      // Always print bounds to keep the graph moving
      Serial.print(0);       
      Serial.print(" ");
      Serial.print(400);     
      Serial.print(" ");
      
      // --- THE STABILITY FILTER ---
      if (frequency >= MIN_FREQ && frequency <= MAX_FREQ) {
        
        // If the new frequency is very close to the last one, increase the counter
        if (abs(frequency - lastFrequency) <= FREQ_TOLERANCE) {
          stableCount++;
        } else {
          // If it suddenly changes, reset the counter to 1
          stableCount = 1;
        }
        
        lastFrequency = frequency; // Save current frequency for the next loop
        
        // Only print if the note has proven it is stable!
        if (stableCount >= REQUIRED_STABLE_READS) {
          Serial.print(frequency); 
          Serial.print("   "); // Add some space before the text
          printNearestNote(frequency); // Text is back!
        } else {
          Serial.println(0); // Not stable yet, print 0
        }
        
      } else {
        // Room is quiet. Reset the filter and drop the graph to 0.
        stableCount = 0;
        lastFrequency = 0.0;
        Serial.println(0); 
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
    samplingActive = !samplingActive;
  }
  previousButtonState = currentButtonState;
}

void collectSamples() {
  unsigned long nextSampleTime = micros();
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    while (micros() < nextSampleTime) {}
    samples[i] = analogRead(micPin);
    nextSampleTime += SAMPLE_PERIOD_US;
  }
}


// --- THE AUTOCORRELATION ALGORITHM --- //
float detectFrequencyAutoCor() {
  
  // 1. FIND THE MIDDLE LINE (DC OFFSET)
  // We need the wave to swing into positive and negative numbers so that 
  // peaks * valleys = negative scores, and peaks * peaks = positive scores.
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += samples[i];
  }
  int dcOffset = sum / SAMPLE_COUNT;
  
  // 2. CONVERT HERTZ TO "LAGS" (ARRAY INDICES)
  // We only care about guitar frequencies (60Hz to 400Hz).
  // To save processing power, we figure out exactly how many array indices 
  // those frequencies represent.
  // Example: 8000 Hz sample rate / 400 Hz highest note = a lag of 20 array slots.
  int minLag = SAMPLE_RATE / MAX_FREQ; 
  int maxLag = SAMPLE_RATE / MIN_FREQ; 
  
  // Variables to keep track of which lag produced the biggest mathematical spike
  int64_t maxCorrelation = 0; 
  int bestLag = -1;

  // 3. SLIDE THE WAVE (THE OUTER LOOP)
  // We test every possible lag distance between our min and max limits.
  for (int lag = minLag; lag <= maxLag; lag++) {
    int64_t correlation = 0;
    
    // 4. MULTIPLY AND SUM (THE INNER LOOP)
    // For the current lag, multiply the original wave against the shifted wave.
    for (int i = 0; i < SAMPLE_COUNT - lag; i++) {
      // Subtracting dcOffset pulls the wave down so it revolves around 0
      int32_t sample1 = samples[i] - dcOffset;
      int32_t sample2 = samples[i + lag] - dcOffset;
      
      // Multiply the points together and add them to the running total
      // We cast to int64_t because multiplying massive audio numbers will 
      // overflow standard 32-bit variables and crash the math.
      correlation += (int64_t)sample1 * sample2; 
    }
    
    // 5. RECORD THE WINNER
    // If this specific lag produced the highest score so far, save it!
    if (correlation > maxCorrelation) {
      maxCorrelation = correlation;
      bestLag = lag;
    }
  }
  
  // 6. CONVERT THE WINNING LAG BACK TO HERTZ
  if (bestLag > 0) {
    // Noise Gate: Even total silence produces a tiny correlation score due to 
    // microscopic electrical noise. If the max score is under 5,000,000, we 
    // assume no strings are actually vibrating and return 0.
    if (maxCorrelation > 5000000) { 
      return (float)SAMPLE_RATE / bestLag;
    }
  }
  
  return 0.0;
}

void printNearestNote(float frequency) {
  const char* noteNames[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
  const float noteFreqs[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63};
  const int noteCount = 6;
  
  int closestIdx = 0;
  float minDiff = abs(frequency - noteFreqs[0]);
  
  for (int i = 1; i < noteCount; i++) {
    float diff = abs(frequency - noteFreqs[i]);
    if (diff < minDiff) {
      minDiff = diff;
      closestIdx = i;
    }
  }
  
  float cents = 1200 * log2(frequency / noteFreqs[closestIdx]);
  
  Serial.print("Note: ");
  Serial.print(noteNames[closestIdx]);
  Serial.print(" (");
  if (cents > 0) Serial.print("+");
  Serial.print(cents, 1);
  Serial.println(" cents)"); // This println finishes the line for the Plotter
}