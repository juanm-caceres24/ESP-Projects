#define LED 2

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
