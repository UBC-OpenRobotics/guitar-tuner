/*
 * Pin Connections:
 * MAX9814 VDD -> 3V3
 * MAX9814 GND -> GND
 * MAX9814 OUT -> GPIO 35
 */


// Change this pin
const int micPin = 35;


void setup() {
  // Baud rate (definitely go higher later)
  Serial.begin(9600);


  // 12 bit ADC (0- 4095)
  analogReadResolution(12);


  // Attentuation from 1.1V --> 3.3V
  analogSetAttenuation(ADC_11db);
}


void loop() {
  int rawValue = analogRead(micPin);


  // Bottom
  Serial.print(0);
  Serial.print(" ");


  // Top
  Serial.print(4095);
  Serial.print(" ");


  // Reading
  Serial.println(rawValue);


  // Small delay, remove once doing FFT
  delayMicroseconds(200);
}
