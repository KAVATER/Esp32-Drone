#include <Arduino.h>

const int ESC_PIN = 33;      // Pin connected to ESC signal
const int POT_PIN = 32;      // Pin connected to potentiometer
const int PWM_FREQ = 50;     // Frequency for ESC control (50Hz)
const int PWM_RES = 12;      // Resolution (12-bit)

void setup() {
  Serial.begin(115200);

  // Configure LEDC PWM using new API
  ledcAttach(ESC_PIN, PWM_FREQ, PWM_RES);  // Automatically assigns a channel

  // Initialize ESC with minimum pulse width (1ms)
  analogWrite(ESC_PIN, 205);               // Send minimum pulse width
  delay(2000);                             // Allow time for ESC initialization
}

void loop() {
  int potValue = analogRead(POT_PIN);      // Read potentiometer value (0-4095)
  
  // Map potentiometer value to ESC pulse width (1ms to 2ms)
  int escPulse = map(potValue, 0, 4095, 205, 410);
  
  analogWrite(ESC_PIN, escPulse);          // Write mapped value to ESC
  delay(20);                               // Short delay for stability

  // Optional: Print values for debugging
  Serial.print("Potentiometer: ");
  Serial.print(potValue);
  Serial.print("\tESC Pulse: ");
  Serial.println(escPulse);
}