/*
 * Frequency Detection with MAX9814 and ESP32 (Goertzel-based)
 *
 * Replaces zero-crossing detection with Goertzel (targeted tone detection).
 * Designed for standard guitar tuning: E2 A2 D3 G3 B3 E4
 *
 * Pin Connections:
 * MAX9814 VDD -> 3V3
 * MAX9814 GND -> GND
 * MAX9814 OUT -> GPIO 35
 * BUTTON -> GPIO 14
 *
 * Notes:
 * - Goertzel is much more robust than zero-crossing in noise and low amplitude.
 * - This implementation:
 *    1) Collects samples
 *    2) Removes DC offset
 *    3) Applies Hann window
 *    4) Computes RMS (amplitude gate)
 *    5) Runs Goertzel around each target frequency (small local sweep)
 *    6) Picks the strongest match and prints frequency + nearest note + cents
 */

 #include <Arduino.h>
 #include <math.h>
 
 // ----------------------- Pins -----------------------
 const int micPin = 35;
 const int buttonPin = 14;
 
 // ---------------- Sampling config -------------------
 const int SAMPLE_RATE = 8000;
 
 // 1024 works; 2048 is steadier for low E (82 Hz) but slower updates.
 // If you can tolerate slower prints, set to 2048.
 const int SAMPLE_COUNT = 1024;
 
 const unsigned long SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;
 const unsigned long PRINT_INTERVAL_MS = 500;
 
 // ---------------- Signal gates ----------------------
 // RMS gate: if signal is below this, ignore (prevents noise/talking from triggering)
 // This value is in ADC counts after DC removal + window. You may need to tune.
 // Start around 60–120 depending on mic gain and placement.
 const float RMS_GATE = 80.0f;
 
 // Confidence gate: require best energy to be this multiple of second-best
 // Higher = more conservative, fewer false positives in noise.
 const float CONF_RATIO_GATE = 1.8f;
 
 // ---------------- Goertzel sweep --------------------
 // We do a small sweep around each target to refine the peak.
 // Wider sweep = more robust but more CPU.
 const float SWEEP_HZ = 8.0f;      // +/- range around each target
 const float SWEEP_STEP_HZ = 0.5f; // step size
 
 // ---------------- Buffers ---------------------------
 int16_t samples[SAMPLE_COUNT];        // raw ADC
 float x[SAMPLE_COUNT];                // float signal after DC removal + window
 float hannWin[SAMPLE_COUNT];          // window
 
 // ---------------- State -----------------------------
 bool samplingActive = false;
 unsigned int currentButtonState;
 unsigned int previousButtonState;
 unsigned long startPress = 0;
 unsigned long lastPrintTime = 0;
 
 // placeholder for dc offset
 int dcOffset = 2048;
 
 // Guitar tuning targets
 const char* noteNames[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
 const float noteFreqs[] = {82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f};
 const int noteCount = 6;
 
 // ----------------------- Helpers -----------------------
 
 static inline float fastAbs(float v) { return v < 0 ? -v : v; }
 
 void buildHannWindow() {
   // Hann window: w[n] = 0.5*(1 - cos(2*pi*n/(N-1)))
   for (int n = 0; n < SAMPLE_COUNT; n++) {
     hannWin[n] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * n / (SAMPLE_COUNT - 1)));
   }
 }
 
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
     while (micros() < nextSampleTime) {
       // wait
     }
     samples[i] = analogRead(micPin);
     nextSampleTime += SAMPLE_PERIOD_US;
   }
 }
 
 void calibrateDCOffset() {
   long sum = 0;
   for (int i = 0; i < SAMPLE_COUNT; i++) sum += samples[i];
   dcOffset = (int)(sum / SAMPLE_COUNT);
 }
 
 float computeRMSAndPrepareSignal() {
   // Remove DC, apply Hann window, compute RMS
   double sumSq = 0.0;
 
   for (int i = 0; i < SAMPLE_COUNT; i++) {
     float v = (float)(samples[i] - dcOffset);
     v *= hannWin[i];        // window
     x[i] = v;
     sumSq += (double)v * (double)v;
   }
 
   float meanSq = (float)(sumSq / (double)SAMPLE_COUNT);
   return sqrtf(meanSq);
 }
 
 // Goertzel "power" at a given frequency (arbitrary freq version)
 float goertzelPower(const float* data, int N, float fs, float targetFreq) {
   // 2nd-order resonator: s[n] = x[n] + 2cos(w)s[n-1] - s[n-2]
   float w = 2.0f * (float)M_PI * (targetFreq / fs);
   float cosine = cosf(w);
   float coeff = 2.0f * cosine;
 
   float s_prev = 0.0f;
   float s_prev2 = 0.0f;
 
   for (int i = 0; i < N; i++) {
     float s = data[i] + coeff * s_prev - s_prev2;
     s_prev2 = s_prev;
     s_prev = s;
   }
 
   // Power estimate:
   // P = s_prev2^2 + s_prev^2 - coeff*s_prev*s_prev2
   float power = s_prev2 * s_prev2 + s_prev * s_prev - coeff * s_prev * s_prev2;
 
   // keep non-negative
   if (power < 0.0f) power = 0.0f;
   return power;
 }
 
 struct PitchResult {
   float freqHz;      // estimated frequency
   int noteIdx;       // which string target we matched best
   float bestPower;   // power at best freq
   float secondPower; // runner-up power across all candidates (for confidence)
   float rms;         // RMS of the frame
   bool confident;    // passes gates
 };
 
 PitchResult detectPitchGoertzel() {
   PitchResult r;
   r.freqHz = 0.0f;
   r.noteIdx = -1;
   r.bestPower = 0.0f;
   r.secondPower = 0.0f;
   r.confident = false;
 
   // 1) DC offset
   calibrateDCOffset();
 
   // 2) Prepare x[] and compute RMS
   r.rms = computeRMSAndPrepareSignal();
   if (r.rms < RMS_GATE) {
     return r; // too quiet
   }
 
   // 3) Sweep around each target, pick global best
   float globalBestP = 0.0f;
   float globalSecondP = 0.0f;
   float globalBestF = 0.0f;
   int globalBestNote = -1;
 
   for (int n = 0; n < noteCount; n++) {
     float base = noteFreqs[n];
 
     float localBestP = 0.0f;
     float localBestF = base;
 
     for (float df = -SWEEP_HZ; df <= SWEEP_HZ + 1e-6f; df += SWEEP_STEP_HZ) {
       float f = base + df;
       if (f < 40.0f || f > 1000.0f) continue; // sanity bounds
 
       float p = goertzelPower(x, SAMPLE_COUNT, (float)SAMPLE_RATE, f);
       if (p > localBestP) {
         localBestP = p;
         localBestF = f;
       }
     }
 
     // update global best/second
     if (localBestP > globalBestP) {
       globalSecondP = globalBestP;
       globalBestP = localBestP;
       globalBestF = localBestF;
       globalBestNote = n;
     } else if (localBestP > globalSecondP) {
       globalSecondP = localBestP;
     }
   }
 
   r.freqHz = globalBestF;
   r.noteIdx = globalBestNote;
   r.bestPower = globalBestP;
   r.secondPower = globalSecondP;
 
   // 4) Confidence gate: best must be sufficiently stronger than runner-up
   // Also ensure we actually found something
   if (r.noteIdx >= 0 && r.bestPower > 0.0f) {
     float ratio = (r.secondPower > 1e-9f) ? (r.bestPower / r.secondPower) : 999.0f;
     if (ratio >= CONF_RATIO_GATE) {
       r.confident = true;
     }
   }
 
   return r;
 }
 
 void printNearestNote(float frequency) {
   // Find closest note among the 6 targets
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
 
 // ----------------------- Setup/Loop -----------------------
 
 void setup() {
   Serial.begin(115200);
 
   // 12-bit ADC (0..4095)
   analogReadResolution(12);
 
   // Attenuation to match higher voltage range
   analogSetAttenuation(ADC_11db);
 
   pinMode(buttonPin, INPUT_PULLUP);
   previousButtonState = digitalRead(buttonPin);
 
   buildHannWindow();
 
   Serial.println("Goertzel Guitar Frequency Detector Ready");
   Serial.println("Press and hold button (>1s) to start/stop sampling");
 }
 
 void loop() {
   handleButton();
 
   if (!samplingActive) return;
 
   unsigned long now = millis();
   if (now - lastPrintTime < PRINT_INTERVAL_MS) return;
 
   collectSamples();
 
   PitchResult res = detectPitchGoertzel();
 
   if (res.rms < RMS_GATE) {
     Serial.print("No pitch (too quiet). RMS=");
     Serial.println(res.rms, 1);
   } else if (!res.confident) {
     Serial.print("Unconfident pitch (noise/voices?). f~");
     Serial.print(res.freqHz, 1);
     Serial.print(" Hz, power ratio=");
     float ratio = (res.secondPower > 1e-9f) ? (res.bestPower / res.secondPower) : 999.0f;
     Serial.println(ratio, 2);
   } else {
     Serial.print("Detected Frequency: ");
     Serial.print(res.freqHz, 1);
     Serial.print(" Hz  |  Matched target: ");
     Serial.print(noteNames[res.noteIdx]);
     Serial.print("  (ratio=");
     float ratio = (res.secondPower > 1e-9f) ? (res.bestPower / res.secondPower) : 999.0f;
     Serial.print(ratio, 2);
     Serial.println(")");
 
     printNearestNote(res.freqHz);
   }
 
   lastPrintTime = now;
 }