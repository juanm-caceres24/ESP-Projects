#include <HardwareSerial.h>

#define UART_TX 17
#define UART_RX 16

HardwareSerial SerialUART(2);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("=== ESP32 UART Test ===");
    SerialUART.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        SerialUART.write(c);
        Serial.print("Sended: ");
        Serial.println(c);
    }
    if (SerialUART.available()) {
        char c = SerialUART.read();
        Serial.print("Received: ");
        Serial.println(c);
    }
}