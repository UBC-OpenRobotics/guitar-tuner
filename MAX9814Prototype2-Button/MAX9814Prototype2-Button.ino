/*
* Pin Connections:
* MAX9814 VDD -> 3V3
* MAX9814 GND -> GND
* MAX9814 OUT -> GPIO 35
* BUTTON -> GPIO 14
*/

const int micPin = 35;
const int buttonPin = 14;

#define ALPHA 1
#define SAMPLING_COUNT 8
#define BAUD_RATE 9600

bool recordingNow;

unsigned int currentButtonState;
unsigned int previousButtonState;

unsigned int voltage;

unsigned int startPress;
unsigned int startRecordingTime;

void setup() {
  // Baud rate (definitely go higher later)
  Serial.begin(BAUD_RATE);

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
    startPress = micros();
  }

  else if (fallingEdge && (micros() - startPress > 1000000)) {
    Serial.println("FALLING EDGE WITH LONG ENOUGH PRESS DETECTED");
    if (!recordingNow) {
      startRecordingTime = micros();
      voltage = 0;
    }

    else if (recordingNow) {
      Serial.print("Average value: ");
      Serial.print(voltage/((micros() - startRecordingTime)/1000000));
      Serial.println(" ADC units");
      Serial.print("Time: ");
      Serial.print((micros() - startRecordingTime)/1000000);
      Serial.println(" seconds");
    }
    recordingNow = !recordingNow;
  }

  else if (fallingEdge) {
    Serial.println("FALLING EDGE NOT LONG ENOUGH");
  }

  if (recordingNow) {
    int smoothedValue = readMic();

    // Bottom
    Serial.print(1024);
    Serial.print(" ");

    // Top
    Serial.print(2048);
    Serial.print(" ");

    // Reading
    Serial.println(smoothedValue);
    voltage += smoothedValue;

    // Small delay, remove once doing FFT
    // delayMicroseconds(200);
  }

  previousButtonState = currentButtonState;
}

int readMic() {
  long sum = 0;
  for (int i = 0; i < SAMPLING_COUNT; i++) {
    sum += analogRead(micPin);
  }
  return sum / SAMPLING_COUNT; 
}