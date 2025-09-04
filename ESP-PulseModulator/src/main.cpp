#include <Arduino.h>

#define LED_0_PIN 12
#define LED_1_PIN 13
#define BUTTON_0_PIN 25
#define BUTTON_1_PIN 26
#define POTENTIOMETER_0_PIN 32
#define POTENTIOMETER_1_PIN 33
#define TIME_IN_US 10 // 10us are 0.01ms
#define DEBOUNCE_DELAY_CYCLES 20000 // Cycles of TIME_IN_US that the button will be ignored (20000 * 10us = 200ms)
#define MIN_PULSE_CYCLES 5000
#define MAX_PULSE_CYCLES 20000

int static debounce_0_counter = 0; // Counter of cycles of TIME_IN_US
int static debounce_1_counter = 0; // Counter of cycles of TIME_IN_US
int static potentiometer_0_value = 0;
int static potentiometer_1_value = 0;
int static led_0_working_state = 1;
int static led_1_working_state = 1;
int static led_0_counter = 0; // Counter of cycles of TIME_IN_US
int static led_1_counter = 0; // Counter of cycles of TIME_IN_US
hw_timer_t static *timer = NULL;

int configPins();
int configInterrupts();
int configTimer();
void button0ISR();
void button1ISR();
void timerISR();
void togglePin(int pin);
void readPotentiometer0();
void readPotentiometer1();
void updateLED0();
void updateLED1();

void setup() {
    Serial.begin(115200);
    configPins();
    configInterrupts();
    configTimer();
}

void loop() {
    readPotentiometer0();
    readPotentiometer1();
    updateLED0();
    updateLED1();
}

int configPins() {
    pinMode(LED_0_PIN, OUTPUT);
    pinMode(LED_1_PIN, OUTPUT);
    pinMode(BUTTON_0_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    return 0;
}

int configInterrupts() {
    attachInterrupt(digitalPinToInterrupt(BUTTON_0_PIN), button0ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), button1ISR, FALLING);
    return 0;
}

int configTimer() {
    timer = timerBegin(0, 80, true);
    timerAlarmWrite(timer, TIME_IN_US, true);
    timerAttachInterrupt(timer, timerISR, true);
    timerAlarmEnable(timer);
    return 0;
}

void button0ISR() {
    if (debounce_0_counter == 0) {
        debounce_0_counter = DEBOUNCE_DELAY_CYCLES;
        led_0_working_state = !led_0_working_state;
        Serial.print("Button 0 pressed, LED 0 working state: ");
        Serial.println(led_0_working_state);
    }
}

void button1ISR() {
    if (debounce_1_counter == 0) {
        debounce_1_counter = DEBOUNCE_DELAY_CYCLES;
        led_1_working_state = !led_1_working_state;
        Serial.print("Button 1 pressed, LED 1 working state: ");
        Serial.println(led_1_working_state);
    }
}

void IRAM_ATTR timerISR() {
    led_0_counter++;
    led_1_counter++;
    if (debounce_0_counter > 0) {
        debounce_0_counter--;
    }
    if (debounce_1_counter > 0) {
        debounce_1_counter--;
    }
}

void togglePin(int pin) {
    digitalWrite(pin, !digitalRead(pin));
}

void readPotentiometer0() {
    potentiometer_0_value = map(analogRead(POTENTIOMETER_0_PIN), 0, 4095, MIN_PULSE_CYCLES, MAX_PULSE_CYCLES);
}

void readPotentiometer1() {
    potentiometer_1_value = map(analogRead(POTENTIOMETER_1_PIN), 0, 4095, MIN_PULSE_CYCLES, MAX_PULSE_CYCLES);
}

void updateLED0() {
    if (led_0_working_state) {
        if (led_0_counter >= potentiometer_0_value) {
            togglePin(LED_0_PIN);
            led_0_counter = 0;
        }
    } else {
        digitalWrite(LED_0_PIN, LOW);
        led_0_counter = 0;
    }
}

void updateLED1() {
    if (led_1_working_state) {
        if (led_1_counter >= potentiometer_1_value) {
            togglePin(LED_1_PIN);
            led_1_counter = 0;
        }
    } else {
        digitalWrite(LED_1_PIN, LOW);
        led_1_counter = 0;
    }
}
