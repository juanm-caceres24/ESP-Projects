#include <Arduino.h>

#define LED_0_PIN 25
#define LED_1_PIN 26
#define BUTTON_0_PIN 32
#define BUTTON_1_PIN 33
#define POTENTIOMETER_0_PIN 34
#define POTENTIOMETER_1_PIN 35
#define REBOUND_DELAY 200
#define TIME_IN_US 10 // 10us are 0.01ms
#define MIN_CYCLES 5000
#define MAX_CYCLES 20000

int static rebound_set_0 = 0;
int static rebound_set_1 = 0;
int static potentiometer_0_value = 0;
int static potentiometer_1_value = 0;
int static led_0_working_state = 0;
int static led_1_working_state = 0;
int static led_0_counter = 0;
int static led_1_counter = 0;
hw_timer_t static *timer = NULL;

int configPins();
int configInterrupts();
int configTimer();
void button0ISR();
void button1ISR();
void timerISR();
void togglePin(int pin);

void setup() {
    Serial.begin(115200);
    configPins();
    configInterrupts();
    configTimer();
}

void loop() {
    potentiometer_0_value = map(analogRead(POTENTIOMETER_0_PIN), 0, 4095, MIN_CYCLES, MAX_CYCLES);
    potentiometer_1_value = map(analogRead(POTENTIOMETER_1_PIN), 0, 4095, MIN_CYCLES, MAX_CYCLES);
    if (led_0_working_state) {
        if (led_0_counter >= potentiometer_0_value) {
            togglePin(LED_0_PIN);
            led_0_counter = 0;
        }
    } else {
        digitalWrite(LED_0_PIN, LOW);
        led_0_counter = 0;
    }
    if (led_1_working_state) {
        if (led_1_counter >= potentiometer_1_value ) {
            togglePin(LED_1_PIN);
            led_1_counter = 0;
        }
    } else {
        digitalWrite(LED_1_PIN, LOW);
        led_1_counter = 0;
    }
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
    if (millis() - rebound_set_0 > REBOUND_DELAY) {
        rebound_set_0 = millis();
        led_0_working_state = !led_0_working_state;
    }
}

void button1ISR() {
    if (millis() - rebound_set_1 > REBOUND_DELAY) {
        rebound_set_1 = millis();
        led_1_working_state = !led_1_working_state;
    }
}

void timerISR() {
    led_0_counter++;
    led_1_counter++;
}

void togglePin(int pin) {
    digitalWrite(pin, !digitalRead(pin));
}
