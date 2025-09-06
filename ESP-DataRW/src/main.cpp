#include <Arduino.h>

#define LED_0_PIN 13
#define LED_1_PIN 12
#define LED_2_PIN 14
#define LED_3_PIN 27
#define LED_4_PIN 26
#define LED_5_PIN 25
#define BUTTON_0_PIN 33
#define BUTTON_1_PIN 32
#define BUTTON_2_PIN 16
#define BUTTON_3_PIN 17
#define BUTTON_4_PIN 5
#define BUTTON_5_PIN 18
#define POTENTIOMETER_0_PIN 35
#define POTENTIOMETER_1_PIN 34
#define TIME_IN_US 10 // 10us are 0.01ms
#define DEBOUNCE_DELAY_CYCLES 20000 // Cycles of TIME_IN_US that the button will be ignored (20000 * 10us = 200ms)
#define DELAY_CYCLES 10000 // Cycles of TIME_IN_US for delay between readings (10000 * 10us = 100ms)
#define DATA_SIZE 256 // Aproximately 2.5 seconds of data at 100ms intervals

int static potentiometer_selection = 0; // 0 = potentiometer 0, 1 = potentiometer 1
int static iteration_status = 0; // 0 = not iterating, 1 = iterating
int static mode_status = 0; // 0 = recording, 1 = playing
int static potentiometer_0_value = 0;
int static potentiometer_1_value = 0;
int static data_0[DATA_SIZE];
int static data_1[DATA_SIZE];
int static data_iterator = 0;
int static delay_counter = 0; // Counter of cycles of TIME_IN_US
int static debounce_counter = 0;
hw_timer_t static *timer = NULL;

void configPins();
void configInterrupts();
void configTimer();
void button0ISR();
void button1ISR();
void button2ISR();
void button3ISR();
void button4ISR();
void button5ISR();
void timerISR();
void readPotentiometers();
void updateLEDs();
void incrementDataIterator();
void setDelayCounter();

void setup() {
    Serial.begin(115200);
    configPins();
    configInterrupts();
    configTimer();
}

void loop() {
    if (iteration_status == 1 && delay_counter == 0) {
        readPotentiometers();
        updateLEDs();
        incrementDataIterator();
        setDelayCounter();
    }
}

void configPins() {
    pinMode(LED_0_PIN, OUTPUT);
    pinMode(LED_1_PIN, OUTPUT);
    pinMode(LED_2_PIN, OUTPUT);
    pinMode(LED_3_PIN, OUTPUT);
    pinMode(LED_4_PIN, OUTPUT);
    pinMode(LED_5_PIN, OUTPUT);
    pinMode(BUTTON_0_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    pinMode(BUTTON_5_PIN, INPUT_PULLUP);
}

void configInterrupts() {
    attachInterrupt(digitalPinToInterrupt(BUTTON_0_PIN), button0ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), button1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_2_PIN), button2ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_3_PIN), button3ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_4_PIN), button4ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_5_PIN), button5ISR, FALLING);
}

void configTimer() {
    timer = timerBegin(0, 80, true);
    timerAlarmWrite(timer, TIME_IN_US, true);
    timerAttachInterrupt(timer, timerISR, true);
    timerAlarmEnable(timer);
}

void button0ISR() {
    if (debounce_counter == 0) {
        debounce_counter = DEBOUNCE_DELAY_CYCLES;
        if (potentiometer_selection == 1) {
            potentiometer_selection = 0;
            Serial.println("Potentiometer 0 selected");
        } else {
            potentiometer_selection = 1;
            Serial.println("Potentiometer 1 selected");
        }
    }
}

void button1ISR() {
    if (debounce_counter == 0) {
        debounce_counter = DEBOUNCE_DELAY_CYCLES;
        if (iteration_status == 1) {
            iteration_status = 0;
            Serial.println("Iteration stopped");
        } else {
            iteration_status = 1;
            Serial.println("Iteration resumed");
        }
    }
}

void button2ISR() {
    if (debounce_counter == 0) {
        debounce_counter = DEBOUNCE_DELAY_CYCLES;
        iteration_status = 0;
        data_iterator = 0;
        if (mode_status == 1) {
            mode_status = 0;
            Serial.println("Recording mode");
        } else {
            mode_status = 1;
            Serial.println("Playing mode");
        }
    }
}

void button3ISR() {
    if (debounce_counter == 0) {
        debounce_counter = DEBOUNCE_DELAY_CYCLES;
        iteration_status = 0;
        data_iterator = 0;
        Serial.println("Data iterator reset to 0");
    }
}

void button4ISR() {
    if (debounce_counter == 0) {
        debounce_counter = DEBOUNCE_DELAY_CYCLES;
        iteration_status = 0;
        data_iterator = 0;
        for (int i = 0; i < DATA_SIZE; i++) {
            data_0[i] = 0;
            data_1[i] = 0;
        }
        Serial.println("Data cleared");
    }
}

void button5ISR() {
    if (debounce_counter == 0) {
        debounce_counter = DEBOUNCE_DELAY_CYCLES;
        // implement here any functionality you want for button 5
        Serial.println("Button 5 pressed");
    }
}

void timerISR() {
    if (delay_counter > 0) {
        delay_counter--;
    }
    if (debounce_counter > 0) {
        debounce_counter--;
    }
}

void incrementDataIterator() {
    data_iterator++;
    if (data_iterator >= DATA_SIZE) {
        data_iterator = 0;
    }
}

void setDelayCounter() {
    delay_counter = DELAY_CYCLES;
}

void readPotentiometers() {
    if (mode_status == 0) { // recording mode
        potentiometer_0_value = map(analogRead(POTENTIOMETER_0_PIN), 0, 4095, 0, 10);
        switch (potentiometer_0_value) {
            case 0:
                data_0[data_iterator] = 0b00000001; // min value
                break;
            case 1:
                data_0[data_iterator] = 0b00000011;
                break;
            case 2:
                data_0[data_iterator] = 0b00000010;
                break;
            case 3:
                data_0[data_iterator] = 0b00000110;
                break;
            case 4:
                data_0[data_iterator] = 0b00000100;
                break;
            case 5:
                data_0[data_iterator] = 0b00001100;
                break;
            case 6:
                data_0[data_iterator] = 0b00001000;
                break;
            case 7:
                data_0[data_iterator] = 0b00011000;
                break;
            case 8:
                data_0[data_iterator] = 0b00010000;
                break;
            case 9:
                data_0[data_iterator] = 0b00110000;
                break;
            case 10:
                data_0[data_iterator] = 0b00100000; // max value
                break;
            default:
                data_0[data_iterator] = 0b11111111; // should not happen
                break;
        }
        potentiometer_1_value = map(analogRead(POTENTIOMETER_1_PIN), 0, 4095, 0, 10);
        switch (potentiometer_1_value) {
            case 0:
                data_1[data_iterator] = 0b00000001; // min value
                break;
            case 1:
                data_1[data_iterator] = 0b00000011;
                break;
            case 2:
                data_1[data_iterator] = 0b00000010;
                break;
            case 3:
                data_1[data_iterator] = 0b00000110;
                break;
            case 4:
                data_1[data_iterator] = 0b00000100;
                break;
            case 5:
                data_1[data_iterator] = 0b00001100;
                break;
            case 6:
                data_1[data_iterator] = 0b00001000;
                break;
            case 7:
                data_1[data_iterator] = 0b00011000;
                break;
            case 8:
                data_1[data_iterator] = 0b00010000;
                break;
            case 9:
                data_1[data_iterator] = 0b00110000;
                break;
            case 10:
                data_1[data_iterator] = 0b00100000; // max value
                break;
            default:
                data_1[data_iterator] = 0b11111111; // should not happen
                break;
        }
    }
}

void updateLEDs() {
    if (potentiometer_selection == 0) { // potentiometer 0
        digitalWrite(LED_0_PIN, (data_0[data_iterator] & 0x01)); // 0b00000001
        digitalWrite(LED_1_PIN, (data_0[data_iterator] & 0x02)); // 0b00000010
        digitalWrite(LED_2_PIN, (data_0[data_iterator] & 0x04)); // 0b00000100
        digitalWrite(LED_3_PIN, (data_0[data_iterator] & 0x08)); // 0b00001000
        digitalWrite(LED_4_PIN, (data_0[data_iterator] & 0x10)); // 0b00010000
        digitalWrite(LED_5_PIN, (data_0[data_iterator] & 0x20)); // 0b00100000
    } else { // potentiometer 1
        digitalWrite(LED_0_PIN, (data_1[data_iterator] & 0x01)); // 0b00000001
        digitalWrite(LED_1_PIN, (data_1[data_iterator] & 0x02)); // 0b00000010
        digitalWrite(LED_2_PIN, (data_1[data_iterator] & 0x04)); // 0b00000100
        digitalWrite(LED_3_PIN, (data_1[data_iterator] & 0x08)); // 0b00001000
        digitalWrite(LED_4_PIN, (data_1[data_iterator] & 0x10)); // 0b00010000
        digitalWrite(LED_5_PIN, (data_1[data_iterator] & 0x20)); // 0b00100000
    }
}
