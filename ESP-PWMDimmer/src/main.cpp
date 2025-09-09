#include <Arduino.h>

#define BUTTON_0_PIN 33
#define BUTTON_1_PIN 32
#define BUTTON_2_PIN 16
#define BUTTON_3_PIN 17
#define BUTTON_4_PIN 5
#define BUTTON_5_PIN 18
#define LED_0_PIN 13
#define LED_1_PIN 12
#define TIME_IN_US 100 // Timer interval in microseconds
#define DEBOUNCE_DELAY_CYCLES 2000 // Cycles of TIME_IN_US that the button will be ignored (2000 * 100us = 200ms)
#define PWM_CYCLES 100 // Number of cycles (of TIME_IN_US) to consider a PWM cycle (100 * 100us = 10ms)
#define MAX_BRIGHTNESS 8

int static debounce_0_counter = 0; // Decrement counter of cycles of TIME_IN_US
int static debounce_1_counter = 0;
int static debounce_2_counter = 0;
int static debounce_3_counter = 0;
int static debounce_4_counter = 0;
int static debounce_5_counter = 0;
int static led_0_working_state = 1; // Working state to know if the general state of the led is ON or OFF
int static led_1_working_state = 1; // Working state to know if the general state of the led is ON or OFF
int static led_0_brightness = MAX_BRIGHTNESS; // Brightness level of LED 0
int static led_1_brightness = MAX_BRIGHTNESS; // Brightness level of LED 1
int static pwm_0_counter = 0; // Counter for PWM cycles of LED 0
int static pwm_1_counter = 0; // Counter for PWM cycles of LED 1
hw_timer_t static *timer = NULL;

int configPins();
int configInterrupts();
int configTimer();
void button0ISR();
void button1ISR();
void button2ISR();
void button3ISR();
void button4ISR();
void button5ISR();
void timerISR();
void updateLED0();
void updateLED1();

void setup() {
    Serial.begin(115200);
    configPins();
    configInterrupts();
    configTimer();
}

void loop() {
    updateLED0();
    updateLED1();
}

int configPins() {
    pinMode(BUTTON_0_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    pinMode(BUTTON_5_PIN, INPUT_PULLUP);
    pinMode(LED_0_PIN, OUTPUT);
    pinMode(LED_1_PIN, OUTPUT);
    return 0;
}

int configInterrupts() {
    attachInterrupt(digitalPinToInterrupt(BUTTON_0_PIN), button0ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), button1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_2_PIN), button2ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_3_PIN), button3ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_4_PIN), button4ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_5_PIN), button5ISR, FALLING);
    return 0;
}

int configTimer() {
    timer = timerBegin(0, 80, true); // Use timer 0, prescaler 80 (1 MHz), count up
    timerAttachInterrupt(timer, timerISR, true); // Attach the interrupt service routine
    timerAlarmWrite(timer, TIME_IN_US, true); // Set alarm to TIME_IN_US microseconds, autoreload true
    timerAlarmEnable(timer); // Enable the alarm
    return 0;
}

void button0ISR() {
    if (debounce_0_counter == 0) {
        debounce_0_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        led_0_working_state = !led_0_working_state; // Toggle the working state
        Serial.print("Button 0 pressed, LED 0 state: ");
        Serial.println(led_0_working_state);
    }
}

void button1ISR() {
    if (debounce_1_counter == 0) {
        debounce_1_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (led_0_brightness > 0) {
            led_0_brightness--;
        }
        Serial.print("Button 1 pressed, LED 0 brightness: ");
        Serial.println(led_0_brightness);
    }
}

void button2ISR() {
    if (debounce_2_counter == 0) {
        debounce_2_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (led_0_brightness < MAX_BRIGHTNESS) {
            led_0_brightness++;
        }
        Serial.print("Button 2 pressed, LED 0 brightness: ");
        Serial.println(led_0_brightness);
    }
}

void button3ISR() {
    if (debounce_3_counter == 0) {
        debounce_3_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        led_1_working_state = !led_1_working_state; // Toggle the working state
        Serial.print("Button 3 pressed, LED 1 state: ");
        Serial.println(led_1_working_state);
    }
}

void button4ISR() {
    if (debounce_4_counter == 0) {
        debounce_4_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (led_1_brightness > 0) {
            led_1_brightness--;
        }
        Serial.print("Button 4 pressed, LED 1 brightness: ");
        Serial.println(led_1_brightness);
    }
}

void button5ISR() {
    if (debounce_5_counter == 0) {
        debounce_5_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (led_1_brightness < MAX_BRIGHTNESS) {
            led_1_brightness++;
        }
        Serial.print("Button 5 pressed, LED 1 brightness: ");
        Serial.println(led_1_brightness);
    }
}

void IRAM_ATTR timerISR() {
    if (debounce_0_counter > 0) {
        debounce_0_counter--; // Decrement the debounce counter
    }
    if (debounce_1_counter > 0) {
        debounce_1_counter--; // Decrement the debounce counter
    }
    if (debounce_2_counter > 0) {
        debounce_2_counter--; // Decrement the debounce counter
    }
    if (debounce_3_counter > 0) {
        debounce_3_counter--; // Decrement the debounce counter
    }
    if (debounce_4_counter > 0) {
        debounce_4_counter--; // Decrement the debounce counter
    }
    if (debounce_5_counter > 0) {
        debounce_5_counter--; // Decrement the debounce counter
    }
    pwm_0_counter++;
    if (pwm_0_counter >= PWM_CYCLES) {
        pwm_0_counter = 0; // Reset the PWM counter after a full cycle
    }
    pwm_1_counter++;
    if (pwm_1_counter >= PWM_CYCLES) {
        pwm_1_counter = 0; // Reset the PWM counter after a full cycle
    }
}

void updateLED0() {
    if (led_0_working_state) {
        if (pwm_0_counter < (led_0_brightness * (PWM_CYCLES / MAX_BRIGHTNESS))) {
            digitalWrite(LED_0_PIN, HIGH);
        } else {
            digitalWrite(LED_0_PIN, LOW);
        }
    } else {
        digitalWrite(LED_0_PIN, LOW);
    }
}

void updateLED1() {
    if (led_1_working_state) {
        if (pwm_1_counter < (led_1_brightness * (PWM_CYCLES / MAX_BRIGHTNESS))) {
            digitalWrite(LED_1_PIN, HIGH);
        } else {
            digitalWrite(LED_1_PIN, LOW);
        }
    } else {
        digitalWrite(LED_1_PIN, LOW);
    }
}
