#include <Arduino.h>
#line 1 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino"
#define LED 2

#line 3 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino"
void setup();
#line 9 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino"
void loop();
#line 3 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino"
void setup() {
	Serial.begin(115200);
	Serial.println("Hello World");
	pinMode(LED, OUTPUT);
}

void loop() {
	Serial.println("Loop");
	digitalWrite(LED, HIGH);
	delay(1000);
	digitalWrite(LED, LOW);
	delay(1000);
}

