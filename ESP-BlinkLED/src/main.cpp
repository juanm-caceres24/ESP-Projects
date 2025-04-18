#include <Arduino.h>

#define LED_BUILTIN 2

uint32_t DELAY_TIME = 1000;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Set delay:");
  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      DELAY_TIME = input.toInt();
      if (DELAY_TIME > 0) {
        break;
      } else {
        Serial.println("Invalid input. Please enter a positive number:");
      }
    }
  }
  Serial.println("Delay set to: " + String(DELAY_TIME) + " ms.");
}

void loop() {
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(DELAY_TIME);
    digitalWrite(LED_BUILTIN, LOW);
    delay(DELAY_TIME);
  }
}
