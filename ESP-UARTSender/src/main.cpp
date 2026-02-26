#include <Arduino.h>

#define RXD1_PIN 16
#define TXD1_PIN 17
#define UART_BAUD 115200
#define START_BYTE 0xAA
#define MSG_TYPE_BUTTON 0x01
#define NUM_BUTTONS 2
#define DEBOUNCE_DELAY 20

uint8_t button_pins[NUM_BUTTONS] = {4, 5};
bool button_state[NUM_BUTTONS];
bool last_button_state[NUM_BUTTONS];
unsigned long last_debounce_time[NUM_BUTTONS];

uint8_t calculate_CRC(uint8_t *data) {
    uint8_t crc = 0;
    for (int i = 0; i < 5; i++) {
        crc ^= data[i];
    }
    return crc;
}

void send_button_event(uint8_t id, bool state) {
    uint8_t packet[6];
    packet[0] = START_BYTE;
    packet[1] = MSG_TYPE_BUTTON;
    packet[2] = id;
    packet[3] = state ? 1 : 0;
    packet[4] = 0;
    packet[5] = calculate_CRC(packet);
    Serial1.write(packet, 6);
}

void setup() {
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pinMode(button_pins[i], INPUT_PULLUP);
        button_state[i] = false;
        last_button_state[i] = false;
        last_debounce_time[i] = 0;
    }
}

void loop() {
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        bool reading = !digitalRead(button_pins[i]);
        if (reading != last_button_state[i]) {
            last_debounce_time[i] = millis();
        }
        if ((millis() - last_debounce_time[i]) > DEBOUNCE_DELAY) {
            if (reading != button_state[i]) {
                button_state[i] = reading;
                send_button_event(i, button_state[i]);
            }
        }
        last_button_state[i] = reading;
    }
}