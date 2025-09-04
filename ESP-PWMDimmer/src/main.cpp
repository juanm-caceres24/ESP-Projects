#include <Arduino.h>

#define BUTTON_0_PIN 32
#define BUTTON_1_PIN 33
#define BUTTON_2_PIN 25
#define BUTTON_3_PIN 26
#define BUTTON_4_PIN 27
#define BUTTON_5_PIN 14
#define LED_0_PIN 12
#define LED_1_PIN 13
#define TIME_IN_US 100 // Timer interval in microseconds
#define CYCLES_OF_BUTTON_DEBOUNCE 5000 // Number of cycles (of TIME_IN_US) to consider a valid button press (5000 * 100us = 500ms)
#define CYCLES_OF_PWM 100 // Number of cycles (of TIME_IN_US) to consider a PWM cycle (100 * 100us = 10ms)
#define MAX_BRIGHTNESS 10

int static debounce_set = 0; // Time when the button was pressed
int static debounce_counter = 0; // Continuous counter similar to millis()
int static led_0_working_state = 1; // Working state to know if the general state of the led is ON or OFF
int static led_1_working_state = 1; // Working state to know if the general state of the led is ON or OFF
int static led_0_brightness = MAX_BRIGHTNESS; // Brightness level of LED 0
int static led_1_brightness = MAX_BRIGHTNESS; // Brightness level of LED 1
int static pwm_counter_0 = 0; // Counter for PWM cycles of LED 0
int static pwm_counter_1 = 0; // Counter for PWM cycles of LED 1
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
    if (debounce_counter - debounce_set > CYCLES_OF_BUTTON_DEBOUNCE) {
        debounce_set = debounce_counter;
        led_0_working_state = !led_0_working_state; // Toggle the working state
        Serial.print("Button 0 pressed, LED 0 state: ");
        Serial.println(led_0_working_state);
    }
}

void button1ISR() {
    if (debounce_counter - debounce_set > CYCLES_OF_BUTTON_DEBOUNCE) {
        debounce_set = debounce_counter;
        if (led_0_brightness > 0) {
            led_0_brightness--;
        }
        Serial.print("Button 1 pressed, LED 0 brightness: ");
        Serial.println(led_0_brightness);
    }
}

void button2ISR() {
    if (debounce_counter - debounce_set > CYCLES_OF_BUTTON_DEBOUNCE) {
        debounce_set = debounce_counter;
        if (led_0_brightness < MAX_BRIGHTNESS) {
            led_0_brightness++;
        }
        Serial.print("Button 2 pressed, LED 0 brightness: ");
        Serial.println(led_0_brightness);
    }
}

void button3ISR() {
    if (debounce_counter - debounce_set > CYCLES_OF_BUTTON_DEBOUNCE) {
        debounce_set = debounce_counter;
        led_1_working_state = !led_1_working_state; // Toggle the working state
        Serial.print("Button 3 pressed, LED 1 state: ");
        Serial.println(led_1_working_state);
    }
}

void button4ISR() {
    if (debounce_counter - debounce_set > CYCLES_OF_BUTTON_DEBOUNCE) {
        debounce_set = debounce_counter;
        if (led_1_brightness > 0) {
            led_1_brightness--;
        }
        Serial.print("Button 4 pressed, LED 1 brightness: ");
        Serial.println(led_1_brightness);
    }
}

void button5ISR() {
    if (debounce_counter - debounce_set > CYCLES_OF_BUTTON_DEBOUNCE) {
        debounce_set = debounce_counter;
        if (led_1_brightness < MAX_BRIGHTNESS) {
            led_1_brightness++;
        }
        Serial.print("Button 5 pressed, LED 1 brightness: ");
        Serial.println(led_1_brightness);
    }
}

void IRAM_ATTR timerISR() {
    debounce_counter++; // Increment the debounce counter every TIME_IN_US
    pwm_counter_0++;
    if (pwm_counter_0 >= CYCLES_OF_PWM) {
        pwm_counter_0 = 0; // Reset the PWM counter after a full cycle
    }
    pwm_counter_1++;
    if (pwm_counter_1 >= CYCLES_OF_PWM) {
        pwm_counter_1 = 0; // Reset the PWM counter after a full cycle
    }
}

void updateLED0() {
    if (led_0_working_state) {
        if (pwm_counter_0 < (led_0_brightness * (CYCLES_OF_PWM / MAX_BRIGHTNESS))) {
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
        if (pwm_counter_1 < (led_1_brightness * (CYCLES_OF_PWM / MAX_BRIGHTNESS))) {
            digitalWrite(LED_1_PIN, HIGH);
        } else {
            digitalWrite(LED_1_PIN, LOW);
        }
    } else {
        digitalWrite(LED_1_PIN, LOW);
    }
}
