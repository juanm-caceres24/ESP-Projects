# 1 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino"


void setup() {
 Serial.begin(115200);
 Serial.println("Hello World");
 pinMode(2, 0x03);
}

void loop() {
 Serial.println("Loop");
 digitalWrite(2, 0x1);
 delay(1000);
 digitalWrite(2, 0x0);
 delay(1000);
}
