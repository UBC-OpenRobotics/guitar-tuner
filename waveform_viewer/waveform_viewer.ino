/*
 * Waveform Viewer with MAX9814 and ESP32
 * 
 * Pin Connections:
 * MAX9814 VDD -> 3V3
 * MAX9814 GND -> GND
 * MAX9814 OUT -> GPIO 35
 * BUTTON -> GPIO 14
 * 
 * Button starts/stops waveform streaming.
 * View waveform using Arduino Serial Plotter (Tools -> Serial Plotter)
 */

// Pin definitions
const int micPin = 35;
const int buttonPin = 14;

// State variables
bool samplingActive = false;
unsigned int currentButtonState;
unsigned int previousButtonState;
unsigned long startPress = 0;

void setup() {
  // Higher baud rate for smooth waveform display
  Serial.begin(115200);
  
  // 12 bit ADC (0-4095)
  analogReadResolution(12);
  
  // Attenuation from 1.1V --> 3.3V
  analogSetAttenuation(ADC_11db);
  
  // Configure button pin as input with pull-up
  pinMode(buttonPin, INPUT_PULLUP);
  
  previousButtonState = digitalRead(buttonPin);
  
  Serial.println("Waveform Viewer Ready");
  Serial.println("Press and hold button (>1s) to start/stop");
}

void loop() {
  // Handle button input
  handleButton();
  
  // If sampling is active, output waveform data
  if (samplingActive) {
    int rawValue = analogRead(micPin);
    
    // Output format for Serial Plotter:
    // Min, Max, and Reading on same line for auto-scaling
    Serial.print(0); // Floor reference
    Serial.print(" ");
    Serial.print(4095); // Ceiling reference
    Serial.print(" ");
    Serial.println(rawValue); // Actual reading
    
    // Small delay for readable waveform (~5kHz sample rate)
    delayMicroseconds(200);
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
      Serial.println("STARTED");
    } else {
      Serial.println("STOPPED");
    }
  }
  
  previousButtonState = currentButtonState;
}
