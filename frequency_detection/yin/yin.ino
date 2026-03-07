/*
 * Frequency Detection with MAX9814 and ESP32 (YIN Pitch Detection)
 *
 * Replaces zero-crossing detection with the YIN method:
 *  - robust against noise / low volume / non-sine waveforms
 *  - outputs a pitch estimate + an internal confidence-like metric (YIN CMND)
 *
 * Pin Connections:
 * MAX9814 VDD -> 3V3
 * MAX9814 GND -> GND
 * MAX9814 OUT -> GPIO 35
 * BUTTON -> GPIO 14
 */

// -------------------- Pin definitions --------------------
const int micPin = 35;
const int buttonPin = 14;

// ---------------- Sampling configuration -----------------
const int SAMPLE_RATE = 8000;
const int SAMPLE_COUNT = 1024;                    // 2048 is steadier for low E but slower
const unsigned long SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;
const unsigned long PRINT_INTERVAL_MS = 500;

// -------------------- YIN configuration ------------------
// Guitar fundamentals: ~82 Hz to ~330 Hz (E2..E4)
// Set a search range; widen a bit for out-of-tune strings if needed.
const float YIN_FMIN = 70.0f;                     // Hz
const float YIN_FMAX = 450.0f;                    // Hz

// Threshold for CMND ("absolute threshold" step). Lower = stricter.
// Typical range: 0.10 .. 0.20
const float YIN_THRESHOLD = 0.15f;

// Signal amplitude gate (RMS) to avoid detecting pitch in noise.
// Tune this for your mic gain/environment.
const float RMS_GATE = 80.0f;

// -------------------- Buffers ----------------------------
// Raw ADC samples
int16_t samples[SAMPLE_COUNT];

// Float signal after DC removal
float x[SAMPLE_COUNT];

// YIN work buffers (size depends on tauMax, but allocate full SAMPLE_COUNT for simplicity)
float yinDiff[SAMPLE_COUNT];
float yinCMND[SAMPLE_COUNT];

// -------------------- State variables --------------------
bool samplingActive = false;
unsigned int currentButtonState;
unsigned int previousButtonState;
unsigned long startPress = 0;
unsigned long lastPrintTime = 0;

// Placeholder DC offset
int dcOffset = 2048;

// -------------------- Helpers ----------------------------
static inline float fastAbs(float v) { return (v < 0.0f) ? -v : v; }

void handleButton() {
  currentButtonState = digitalRead(buttonPin);

  bool risingEdge = !currentButtonState && previousButtonState;
  bool fallingEdge = currentButtonState && !previousButtonState;

  if (risingEdge) {
    startPress = millis();
  } else if (fallingEdge && (millis() - startPress > 1000)) {
    samplingActive = !samplingActive;

    if (samplingActive) {
      Serial.println("\n=== SAMPLING STARTED ===");
      lastPrintTime = 0;
    } else {
      Serial.println("\n=== SAMPLING STOPPED ===");
    }
  }

  previousButtonState = currentButtonState;
}

void collectSamples() {
  unsigned long nextSampleTime = micros();

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    while (micros() < nextSampleTime) { /* wait */ }
    samples[i] = analogRead(micPin);
    nextSampleTime += SAMPLE_PERIOD_US;
  }
}

void calibrateDCOffset() {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += samples[i];
  }
  dcOffset = (int)(sum / SAMPLE_COUNT);
}

float prepareSignalAndRMS() {
  // Remove DC and compute RMS
  double sumSq = 0.0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float v = (float)(samples[i] - dcOffset);
    x[i] = v;
    sumSq += (double)v * (double)v;
  }
  float meanSq = (float)(sumSq / (double)SAMPLE_COUNT);
  return sqrtf(meanSq);
}

// Parabolic interpolation around tau to get sub-sample accuracy
// Uses CMND values at tau-1, tau, tau+1
float parabolicInterpTau(int tau) {
  float x0 = yinCMND[tau - 1];
  float x1 = yinCMND[tau];
  float x2 = yinCMND[tau + 1];

  float denom = (2.0f * x1 - x2 - x0);
  if (fastAbs(denom) < 1e-12f) return (float)tau;

  // Vertex offset from tau (in samples)
  float delta = 0.5f * (x2 - x0) / denom;
  return (float)tau + delta;
}

// YIN pitch detection.
// Returns frequency in Hz; returns 0 if no confident pitch found.
// Also optionally returns the CMND value at the selected tau (lower is better).
float detectFrequencyYIN(float* outCMND = nullptr) {
  // Gate by amplitude first
  float rms = prepareSignalAndRMS();
  if (rms < RMS_GATE) {
    if (outCMND) *outCMND = 1.0f;
    return 0.0f;
  }

  // Convert frequency range to lag (tau) range
  int tauMin = (int)floorf((float)SAMPLE_RATE / YIN_FMAX);
  int tauMax = (int)ceilf((float)SAMPLE_RATE / YIN_FMIN);

  // Safety clamp
  if (tauMin < 2) tauMin = 2;
  if (tauMax > SAMPLE_COUNT / 2) tauMax = SAMPLE_COUNT / 2; // need room: N - tau

  // 1) Difference function d(tau)
  // d(tau) = sum_{j=0}^{N-tau-1} (x[j] - x[j+tau])^2
  yinDiff[0] = 0.0f;
  for (int tau = 1; tau <= tauMax; tau++) {
    double sum = 0.0;
    int limit = SAMPLE_COUNT - tau;
    for (int j = 0; j < limit; j++) {
      float delta = x[j] - x[j + tau];
      sum += (double)delta * (double)delta;
    }
    yinDiff[tau] = (float)sum;
  }

  // 2) Cumulative Mean Normalized Difference (CMND)
  // cmnd(tau) = d(tau) / ((1/tau) * sum_{k=1..tau} d(k))
  yinCMND[0] = 1.0f;
  double runningSum = 0.0;
  for (int tau = 1; tau <= tauMax; tau++) {
    runningSum += (double)yinDiff[tau];
    if (runningSum < 1e-12) {
      yinCMND[tau] = 1.0f;
    } else {
      double mean = runningSum / (double)tau;
      yinCMND[tau] = (float)((double)yinDiff[tau] / mean);
    }
  }

  // 3) Absolute threshold + first minimum search
  int tauEstimate = -1;

  // Find first tau in [tauMin, tauMax] where cmnd < threshold,
  // then walk forward to the local minimum.
  for (int tau = tauMin; tau <= tauMax; tau++) {
    if (yinCMND[tau] < YIN_THRESHOLD) {
      tauEstimate = tau;
      // refine to local min
      while (tauEstimate + 1 <= tauMax && yinCMND[tauEstimate + 1] < yinCMND[tauEstimate]) {
        tauEstimate++;
      }
      break;
    }
  }

  if (tauEstimate < 0) {
    // No dip crossed the threshold -> not confident
    if (outCMND) *outCMND = 1.0f;
    return 0.0f;
  }

  // 4) Parabolic interpolation (needs neighbors)
  float refinedTau = (float)tauEstimate;
  if (tauEstimate > 1 && tauEstimate < tauMax) {
    refinedTau = parabolicInterpTau(tauEstimate);
  }

  float cmndVal = yinCMND[tauEstimate];
  if (outCMND) *outCMND = cmndVal;

  if (refinedTau <= 0.0f) return 0.0f;

  // Frequency estimate
  float freq = (float)SAMPLE_RATE / refinedTau;

  // Sanity check: must land inside the band we searched (with slight slack)
  if (freq < (YIN_FMIN * 0.8f) || freq > (YIN_FMAX * 1.2f)) {
    return 0.0f;
  }

  return freq;
}

void printNearestNote(float frequency) {
  // Standard guitar tuning frequencies (E2 A2 D3 G3 B3 E4)
  const char* noteNames[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
  const float noteFreqs[] = {82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f};
  const int noteCount = 6;

  int closestIdx = 0;
  float minDiff = fabsf(frequency - noteFreqs[0]);

  for (int i = 1; i < noteCount; i++) {
    float diff = fabsf(frequency - noteFreqs[i]);
    if (diff < minDiff) {
      minDiff = diff;
      closestIdx = i;
    }
  }

  float cents = 1200.0f * log2f(frequency / noteFreqs[closestIdx]);

  Serial.print("  Nearest note: ");
  Serial.print(noteNames[closestIdx]);
  Serial.print(" (");
  Serial.print(noteFreqs[closestIdx], 2);
  Serial.print(" Hz), ");

  if (cents > 0) Serial.print("+");
  Serial.print(cents, 1);
  Serial.println(" cents");
}

// -------------------- Arduino setup/loop -----------------

void setup() {
  Serial.begin(115200);

  // 12-bit ADC (0-4095)
  analogReadResolution(12);

  // Attenuation from 1.1V --> 3.3V
  analogSetAttenuation(ADC_11db);

  pinMode(buttonPin, INPUT_PULLUP);
  previousButtonState = digitalRead(buttonPin);

  Serial.println("Frequency Detector Ready (YIN)");
  Serial.println("Press and hold button (>1s) to start/stop sampling");
}

void loop() {
  handleButton();

  if (!samplingActive) return;

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime < PRINT_INTERVAL_MS) return;

  collectSamples();
  calibrateDCOffset();

  float cmnd = 1.0f;
  float frequency = detectFrequencyYIN(&cmnd);

  if (frequency > 0.0f) {
    Serial.print("Detected Frequency: ");
    Serial.print(frequency, 2);
    Serial.print(" Hz  |  YIN CMND=");
    Serial.println(cmnd, 3); // lower is better; <~0.15 often good

    printNearestNote(frequency);
  } else {
    Serial.print("No frequency detected (quiet/no periodicity).");
    Serial.print("  RMS gate=");
    Serial.print(RMS_GATE, 1);
    Serial.print("  YIN threshold=");
    Serial.println(YIN_THRESHOLD, 2);
  }

  lastPrintTime = currentTime;
}