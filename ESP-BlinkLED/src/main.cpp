#include <Arduino.h>

#define LED_BUILTIN 2
#define DELAY_TIME 200

void initializeSerial();
void initializeGPIO(int GPIO_pin);
void blinkPin(int GPIO_pin, int delayTime);

void setup() {
  initializeSerial();
  initializeGPIO(LED_BUILTIN);
  Serial.println("Setup complete");
}

void loop() {
  blinkPin(LED_BUILTIN, DELAY_TIME);
  Serial.println("LED blinked");
}

void initializeSerial() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Serial initialized");
}

void initializeGPIO(int GPIO_pin) {
  pinMode(GPIO_pin, OUTPUT);
  digitalWrite(GPIO_pin, LOW);
  Serial.println("GPIO initialized");
}

void blinkPin(int GPIO_pin, int delayTime) {
  digitalWrite(GPIO_pin, HIGH);
  delay(delayTime);
  digitalWrite(GPIO_pin, LOW);
  delay(delayTime);
}
