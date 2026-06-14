#include <Arduino.h>

#define RXD1_PIN 16
#define TXD1_PIN 17
#define UART_BAUD 115200
#define START_BYTE 0xAA
#define MSG_TYPE_BUTTON 0x01
#define NUM_BUTTONS 2
#define DEBOUNCE_DELAY 20
#define KEYPAD_ROWS 3
#define KEYPAD_COLS 4
#define KEYPAD_BUTTON_COUNT (KEYPAD_ROWS * KEYPAD_COLS)
#define KEYPAD_DEBOUNCE_MS 20
#define MATRIX_BUTTON_OFFSET NUM_BUTTONS

uint8_t button_pins[NUM_BUTTONS] = {13, 32};
bool button_state[NUM_BUTTONS];
bool last_button_state[NUM_BUTTONS];
unsigned long last_debounce_time[NUM_BUTTONS];
const uint8_t keypad_row_pins[KEYPAD_ROWS] = {14, 27, 26}; 
const uint8_t keypad_col_pins[KEYPAD_COLS] = {25, 33, 18, 19};
bool keypad_debounced_state[KEYPAD_BUTTON_COUNT] = {false};
bool keypad_last_raw_state[KEYPAD_BUTTON_COUNT] = {false};
uint32_t keypad_last_change_ms[KEYPAD_BUTTON_COUNT] = {0};

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

void keypad_init() {
    for (uint8_t i = 0; i < KEYPAD_ROWS; i++) {
        pinMode(keypad_row_pins[i], OUTPUT);
        digitalWrite(keypad_row_pins[i], HIGH);
    }
    for (uint8_t i = 0; i < KEYPAD_COLS; i++) {
        pinMode(keypad_col_pins[i], INPUT_PULLUP);
    }
}

uint32_t scan_keypad_raw_mask() {
    uint32_t mask = 0;
    for (uint8_t row = 0; row < KEYPAD_ROWS; row++) {
        digitalWrite(keypad_row_pins[row], LOW);
        delayMicroseconds(3);
        for (uint8_t col = 0; col < KEYPAD_COLS; col++) {
            if (digitalRead(keypad_col_pins[col]) == LOW) {
                mask |= (1u << ((row * KEYPAD_COLS) + col));
            }
        }
        digitalWrite(keypad_row_pins[row], HIGH);
    }
    return mask;
}

void update_keypad_buttons() {
    const uint32_t now_ms = millis();
    const uint32_t raw_mask = scan_keypad_raw_mask();
    for (uint8_t i = 0; i < KEYPAD_BUTTON_COUNT; i++) {
        const bool raw_pressed = (raw_mask & (1u << i)) != 0;
        if (raw_pressed != keypad_last_raw_state[i]) {
            keypad_last_raw_state[i] = raw_pressed;
            keypad_last_change_ms[i] = now_ms;
        }
        if ((uint32_t)(now_ms - keypad_last_change_ms[i]) >= KEYPAD_DEBOUNCE_MS) {
            if (keypad_debounced_state[i] != raw_pressed) {
                keypad_debounced_state[i] = raw_pressed;
                send_button_event(MATRIX_BUTTON_OFFSET + i, keypad_debounced_state[i]);
            }
        }
    }
}

void setup() {
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pinMode(button_pins[i], INPUT_PULLUP);
        button_state[i] = false;
        last_button_state[i] = false;
        last_debounce_time[i] = 0;
    }
    keypad_init();
}

void loop() {
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        bool reading = digitalRead(button_pins[i]);
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
    update_keypad_buttons();
}
