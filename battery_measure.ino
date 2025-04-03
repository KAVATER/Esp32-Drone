const int BATTERY_PIN = 35; // ADC pin connected to voltage divider
const float R1 = 10000.0; // Resistor values in ohms
const float R2 = 2200.0;
const float ADC_MAX = 4095.0; // Maximum ADC value (12-bit resolution)
const float VOLTAGE_REF = 3.3; // Reference voltage for ADC

void setup() {
  Serial.begin(115200);

}

void loop() {
  int adcValue = analogRead(BATTERY_PIN); // Read ADC value
  float vOut = (adcValue / ADC_MAX) * VOLTAGE_REF; // Convert ADC value to voltage
  float vIn = vOut * (R1 + R2) / R2; // Calculate actual battery voltage

  Serial.print("Battery Voltage: ");
  Serial.print(vIn);
  Serial.println(" V");

  delay(200); // Delay for readability
}
