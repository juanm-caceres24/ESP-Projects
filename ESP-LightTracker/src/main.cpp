#include <Arduino.h>

#define BUTTON_0_PIN 33
#define BUTTON_1_PIN 32
#define BUTTON_2_PIN 16
#define BUTTON_3_PIN 17
//#define BUTTON_4_PIN 5
//#define BUTTON_5_PIN 18

#define PWM_0_PIN 13
#define PWM_1_PIN 12
#define MOTOR_0_DIRECTION_0_PIN 14
#define MOTOR_0_DIRECTION_1_PIN 27
#define MOTOR_1_DIRECTION_0_PIN 26
#define MOTOR_1_DIRECTION_1_PIN 25

#define TIME_IN_US 100 // Timer interval in microseconds
#define DEBOUNCE_DELAY_CYCLES 2000 // Cycles of TIME_IN_US that the button will be ignored (2000 * 100us = 200ms)
#define PWM_CYCLES 100 // Number of cycles (of TIME_IN_US) to consider a PWM cycle (100 * 100us = 10ms)

#define MAX_THROTTLE 4 // Maximum throttle level

int static debounce_0_counter = 0; // Decrement counter of cycles of TIME_IN_US
int static debounce_1_counter = 0;
int static debounce_2_counter = 0;
int static debounce_3_counter = 0;
int static motor_0_working_state = 0; // Working state of Motor 0 (0 = off, 1 = on)
int static motor_1_working_state = 0; // Working state of Motor 1 (0 = off, 1 = on)
int static motor_0_direction = 0; // Direction of Motor 0
int static motor_1_direction = 0; // Direction of Motor 1
int static motor_0_throttle = 0; // Throttle level of Motor 0
int static motor_1_throttle = 0; // Throttle level of Motor 1
int static pwm_0_counter = 0; // Counter for PWM cycles of Motor 0
int static pwm_1_counter = 0; // Counter for PWM cycles of Motor 1
hw_timer_t static *timer = NULL;

int configPins();
int configInterrupts();
int configTimer();
void button0ISR();
void button1ISR();
void button2ISR();
void button3ISR();
void timerISR();
void updateMotor0();
void updateMotor1();

void setup() {
    Serial.begin(115200);
    configPins();
    configInterrupts();
    configTimer();
}

void loop() {
    updateMotor0();
    updateMotor1();
}

int configPins() {
    pinMode(BUTTON_0_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    pinMode(BUTTON_5_PIN, INPUT_PULLUP);
    pinMode(PWM_0_PIN, OUTPUT);
    pinMode(PWM_1_PIN, OUTPUT);
    pinMode(MOTOR_0_DIRECTION_0_PIN, OUTPUT);
    pinMode(MOTOR_0_DIRECTION_1_PIN, OUTPUT);
    pinMode(MOTOR_1_DIRECTION_0_PIN, OUTPUT);
    pinMode(MOTOR_1_DIRECTION_1_PIN, OUTPUT);
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
        motor_0_working_state = !motor_0_working_state; // Toggle the working state
        Serial.print("Button 0 pressed, Motor 0 state: ");
        Serial.println(motor_0_working_state);
    }
}

void button1ISR() {
    if (debounce_1_counter == 0) {
        debounce_1_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (motor_0_direction == 0) { // If direction is forward
            if (motor_0_throttle > 0) {
                motor_0_throttle--; // Decrease throttle
            } else {
                motor_0_direction = 1; // Change direction to reverse if throttle is zero
            }
        } else { // If direction is reverse
            if (motor_0_throttle < MAX_THROTTLE) {
                motor_0_throttle++; // Increase throttle
            }
        }
        Serial.print("Button 1 pressed, Motor 0 throttle: ");
        Serial.print(motor_0_throttle);
        Serial.print(", Direction: ");
        Serial.println(motor_0_direction == 0 ? "Forward" : "Reverse");
    }
}

void button2ISR() {
    if (debounce_2_counter == 0) {
        debounce_2_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (motor_0_direction == 0) { // If direction is forward
            if (motor_0_throttle < MAX_THROTTLE) {
                motor_0_throttle++; // Increase throttle
            }
        } else { // If direction is reverse
            if (motor_0_throttle > 0) {
                motor_0_throttle--; // Decrease throttle
            } else {
                motor_0_direction = 0; // Change direction to forward if throttle is zero
            }
        }
        Serial.print("Button 2 pressed, Motor 0 throttle: ");
        Serial.println(motor_0_throttle);
        Serial.print(", Direction: ");
        Serial.println(motor_0_direction == 0 ? "Forward" : "Reverse");
    }
}

void button3ISR() {
    if (debounce_3_counter == 0) {
        debounce_3_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        motor_1_working_state = !motor_1_working_state; // Toggle the working state
        Serial.print("Button 3 pressed, Motor 1 state: ");
        Serial.println(motor_1_working_state);
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
    pwm_0_counter++;
    if (pwm_0_counter >= PWM_CYCLES) {
        pwm_0_counter = 0; // Reset the PWM counter after a full cycle
    }
    pwm_1_counter++;
    if (pwm_1_counter >= PWM_CYCLES) {
        pwm_1_counter = 0; // Reset the PWM counter after a full cycle
    }
}

void updateMotor0() {
    if (motor_0_direction) {
        digitalWrite(MOTOR_0_DIRECTION_0_PIN, LOW);
        digitalWrite(MOTOR_0_DIRECTION_1_PIN, HIGH);
    } else {
        digitalWrite(MOTOR_0_DIRECTION_0_PIN, HIGH);
        digitalWrite(MOTOR_0_DIRECTION_1_PIN, LOW);
    }
    if (motor_0_working_state) {
        if (pwm_0_counter < (motor_0_throttle * (PWM_CYCLES / MAX_THROTTLE))) {
            digitalWrite(PWM_0_PIN, HIGH);
        } else {
            digitalWrite(PWM_0_PIN, LOW);
        }
    } else {
        digitalWrite(PWM_0_PIN, LOW);
    }
}

void updateMotor1() {
    if (motor_1_direction) {
        digitalWrite(MOTOR_1_DIRECTION_0_PIN, LOW);
        digitalWrite(MOTOR_1_DIRECTION_1_PIN, HIGH);
    } else {
        digitalWrite(MOTOR_1_DIRECTION_0_PIN, HIGH);
        digitalWrite(MOTOR_1_DIRECTION_1_PIN, LOW);
    }
    if (motor_1_working_state) {
        if (pwm_1_counter < (motor_1_throttle * (PWM_CYCLES / MAX_THROTTLE))) {
            digitalWrite(PWM_1_PIN, HIGH);
        } else {
            digitalWrite(PWM_1_PIN, LOW);
        }
    } else {
        digitalWrite(PWM_1_PIN, LOW);
    }
}
