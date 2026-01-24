/*
* Pin Connections:
* MAX9814 VDD -> 3V3
* MAX9814 GND -> GND
* MAX9814 OUT -> GPIO 35
* BUTTON -> GPIO 14
*/

const int micPin = 35;
const int buttonPin = 14;

bool recordingNow;

unsigned int currentButtonState;
unsigned int previousButtonState;

unsigned int voltage;

unsigned int startPress;
unsigned int startRecordingTime;

void setup() {
  // Baud rate (definitely go higher later)
  Serial.begin(9600);

  // 12 bit ADC (0 - 4095)
  // ADC Unit / 4095 * 3.3 = Voltage
  analogReadResolution(12);

  // Attentuation from 1.1V --> 3.3V
  analogSetAttenuation(ADC_11db);

  recordingNow = false;
}


void loop() {
  currentButtonState = digitalRead(buttonPin);
  // Serial.print("Current button state: ");
  // Serial.println(currentButtonState);

  bool risingEdge = !currentButtonState && previousButtonState;
  bool fallingEdge = currentButtonState && !previousButtonState;

  
  if (risingEdge) {
    Serial.println("RISING EDGE DETECTED");
    startPress = millis();
  }

  else if (fallingEdge && (millis() - startPress > 1000)) {
    Serial.println("FALLING EDGE WITH LONG ENOUGH PRESS DETECTED");
    if (!recordingNow) {
      startRecordingTime = millis();
      voltage = 0;
    }

    else if (recordingNow) {
      Serial.print("Average value: ");
      Serial.print(voltage/((millis() - startRecordingTime)/1000));
      Serial.println(" ADC units");
      Serial.print("Time: ");
      Serial.print((millis() - startRecordingTime)/1000);
      Serial.println(" seconds");
    }
    recordingNow = !recordingNow;
  }

  else if (fallingEdge) {
    Serial.println("FALLING EDGE NOT LONG ENOUGH");
  }

  if (recordingNow) {
    int rawValue = analogRead(micPin);

    // Bottom
    Serial.print(0);
    Serial.print(" ");

    // Top
    Serial.print(4095);
    Serial.print(" ");

    // Reading
    Serial.println(rawValue);
    voltage += rawValue;

    // Small delay, remove once doing FFT
    delayMicroseconds(200);
  }

  previousButtonState = currentButtonState;
}