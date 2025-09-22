#include <Arduino.h>

#define BUTTON_0_PIN            16          // Enable/Disable Button
#define BUTTON_1_PIN            17
#define BUTTON_2_PIN            5
#define BUTTON_3_PIN            18

#define LDR_0_PIN               33          // Right LDR
#define LDR_1_PIN               32          // Left LDR
#define LDR_2_PIN               35          // Up LDR
#define LDR_3_PIN               34          // Down LDR

#define PWM_0_PIN               13          // Right-Left Motor PWM pin
#define MOTOR_0_DIRECTION_0_PIN 14
#define MOTOR_0_DIRECTION_1_PIN 27

#define PWM_1_PIN               12          // Up-Down Motor PWM pin
#define MOTOR_1_DIRECTION_0_PIN 26
#define MOTOR_1_DIRECTION_1_PIN 25

#define MAX_THROTTLE            32          // Maximum throttle level

#define TIME_IN_US              100         // Timer interval in microseconds (0.1ms)
#define DEBOUNCE_DELAY_CYCLES   2000        // Cycles of TIME_IN_US that the button will be ignored (2000 * 0.1ms = 200ms)
#define PWM_CYCLES              100         // Number of cycles (of TIME_IN_US) to consider a PWM cycle (100 * 0.1ms = 10ms)

#define KP_0                    0.01        // Proportional gain for the control algorithm
#define KI_0                    0.00007     // Integral gain for the control algorithm
#define KD_0                    0.00001     // Derivative gain for the control algorithm
#define WINDUP_LIMIT_0          1000        // Integral windup limit for Motor 0

#define KP_1                    0.01        // Proportional gain for the control algorithm
#define KI_1                    0.00007     // Integral gain for the control algorithm
#define KD_1                    0.00001     // Derivative gain for the control algorithm
#define WINDUP_LIMIT_1          1000        // Integral windup limit for Motor 1

int static debounce_0_counter = 0; // Counter for debouncing Button 0
int static debounce_1_counter = 0; // Counter for debouncing Button 1
int static debounce_2_counter = 0; // Counter for debouncing Button 2
int static debounce_3_counter = 0; // Counter for debouncing Button 3

int static ldr_0_value = 0; // Value read from LDR 0
int static ldr_1_value = 0; // Value read from LDR 1
int static ldr_2_value = 0; // Value read from LDR 2
int static ldr_3_value = 0; // Value read from LDR 3

int static motor_0_enable = 0; // Enable state of Motor 0 (0 = off, 1 = on)
int static motor_0_direction = 0; // Direction of Motor 0
int static motor_0_throttle = 0; // Throttle level of Motor 0
int static pwm_0_counter = 0; // Counter for PWM cycles of Motor 0

int static motor_1_enable = 0; // Enable state of Motor 1 (0 = off, 1 = on)
int static motor_1_direction = 0; // Direction of Motor 1
int static motor_1_throttle = 0; // Throttle level of Motor 1
int static pwm_1_counter = 0; // Counter for PWM cycles of Motor 1

float static setpoint_0 = 0; // Desired value for Motor 0 control
float static measured_value_0 = 0; // Measured value for Motor 0 control
float static error_0 = 0; // Current error for Motor 0 control
float static previous_error_0 = 0; // Previous error for Motor 0 control
float static integral_0 = 0; // Integral term for Motor 0 control
float static derivative_0 = 0; // Derivative term for Motor 0 control
float static output_0 = 0; // PID output for Motor 0 control

float static setpoint_1 = 0; // Desired value for Motor 1 control
float static measured_value_1 = 0; // Measured value for Motor 1 control
float static error_1 = 0; // Current error for Motor 1 control
float static previous_error_1 = 0; // Previous error for Motor 1 control
float static integral_1 = 0; // Integral term for Motor 1 control
float static derivative_1 = 0; // Derivative term for Motor 1 control
float static output_1 = 0; // PID output for Motor 1 control

hw_timer_t static *timer = NULL; // Hardware timer

void configPins();
void configInterrupts();
void configTimer();
void button0ISR();
void button1ISR();
void button2ISR();
void button3ISR();
void timerISR();
void readLDRs();
void processThrottleAndDirection();
int calculatePID_0();
int calculatePID_1();
void updateMotor0();
void updateMotor1();

void setup() {
    Serial.begin(115200);
    configPins();
    configInterrupts();
    configTimer();
}

void loop() {
    readLDRs();
    processThrottleAndDirection();
    updateMotor0();
    updateMotor1();
}

void configPins() {
    pinMode(BUTTON_0_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);

    pinMode(LDR_0_PIN, INPUT);
    pinMode(LDR_1_PIN, INPUT);
    pinMode(LDR_2_PIN, INPUT);
    pinMode(LDR_3_PIN, INPUT);

    pinMode(PWM_0_PIN, OUTPUT);
    pinMode(MOTOR_0_DIRECTION_0_PIN, OUTPUT);
    pinMode(MOTOR_0_DIRECTION_1_PIN, OUTPUT);

    pinMode(PWM_1_PIN, OUTPUT);
    pinMode(MOTOR_1_DIRECTION_0_PIN, OUTPUT);
    pinMode(MOTOR_1_DIRECTION_1_PIN, OUTPUT);
}

void configInterrupts() {
    attachInterrupt(digitalPinToInterrupt(BUTTON_0_PIN), button0ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), button1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_2_PIN), button2ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_3_PIN), button3ISR, FALLING);
}

void configTimer() {
    timer = timerBegin(0, 80, true); // Use timer 0, prescaler 80 (1 MHz), count up
    timerAttachInterrupt(timer, timerISR, true); // Attach the interrupt service routine
    timerAlarmWrite(timer, TIME_IN_US, true); // Set alarm to TIME_IN_US microseconds, autoreload true
    timerAlarmEnable(timer); // Enable the alarm
}

void button0ISR() {
    if (debounce_0_counter == 0) {
        debounce_0_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        motor_0_enable = !motor_0_enable; // Toggle the enable state of Motor 0
        Serial.print("Button 0 pressed, Motor 0 state: ");
        Serial.println(motor_0_enable);
    }
}

void button1ISR() {
    if (debounce_1_counter == 0) {
        debounce_1_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        motor_1_enable = !motor_1_enable; // Toggle the enable state of Motor 1
        Serial.print("Button 1 pressed, Motor 1 state: ");
        Serial.println(motor_1_enable);
    }
}

void button2ISR() {
    if (debounce_2_counter == 0) {
        debounce_2_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        
        // DO NOTHING, THIS BUTTON IS NOT USED
    }
}

void button3ISR() {
    if (debounce_3_counter == 0) {
        debounce_3_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        
        // DO NOTHING, THIS BUTTON IS NOT USED
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

void readLDRs() {
    ldr_0_value = analogRead(LDR_0_PIN);
    ldr_1_value = analogRead(LDR_1_PIN);
    ldr_2_value = analogRead(LDR_2_PIN);
    ldr_3_value = analogRead(LDR_3_PIN);
}

void processThrottleAndDirection() {
    int diff_right_left = ldr_0_value - ldr_1_value;
    if (diff_right_left > 0) {
        motor_0_direction = 0; // Right
    } else {
        motor_0_direction = 1; // Left
    }
    motor_0_throttle = calculatePID_0(); // Use PID to determine throttle

    int diff_up_down = ldr_2_value - ldr_3_value;
    if (diff_up_down > 0) {
        motor_1_direction = 0; // Up
    } else {
        motor_1_direction = 1; // Down
    }
    motor_1_throttle = calculatePID_1(); // Use PID to determine throttle
}

int calculatePID_0() {
    measured_value_0 = ldr_0_value - ldr_1_value; // Current value (difference between LDRs)
    error_0 = setpoint_0 - measured_value_0;
    integral_0 += error_0 * (TIME_IN_US / 1000000.0); // Integrate the error over time
    integral_0 = constrain(integral_0, -WINDUP_LIMIT_0, WINDUP_LIMIT_0); // Prevent integral windup
    derivative_0 = (error_0 - previous_error_0) / (TIME_IN_US / 1000000.0); // Derivative of the error
    output_0 = KP_0 * error_0 + KI_0 * integral_0 + KD_0 * derivative_0;
    previous_error_0 = error_0;
    return min(MAX_THROTTLE, max(1, abs((int)output_0))); // Scale throttle based on PID output
}

int calculatePID_1() {
    measured_value_1 = ldr_2_value - ldr_3_value; // Current value (difference between LDRs)
    error_1 = setpoint_1 - measured_value_1;
    integral_1 += error_1 * (TIME_IN_US / 1000000.0); // Integrate the error over time
    integral_1 = constrain(integral_1, -WINDUP_LIMIT_1, WINDUP_LIMIT_1); // Prevent integral windup
    derivative_1 = (error_1 - previous_error_1) / (TIME_IN_US / 1000000.0); // Derivative of the error
    output_1 = KP_1 * error_1 + KI_1 * integral_1 + KD_1 * derivative_1;
    previous_error_1 = error_1;
    return min(MAX_THROTTLE, max(1, abs((int)output_1))); // Scale throttle based on PID output
}

void updateMotor0() {
    if (motor_0_direction) {
        digitalWrite(MOTOR_0_DIRECTION_0_PIN, LOW);
        digitalWrite(MOTOR_0_DIRECTION_1_PIN, HIGH);
    } else {
        digitalWrite(MOTOR_0_DIRECTION_0_PIN, HIGH);
        digitalWrite(MOTOR_0_DIRECTION_1_PIN, LOW);
    }
    if (motor_0_enable) {
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
    if (motor_1_enable) {
        if (pwm_1_counter < (motor_1_throttle * (PWM_CYCLES / MAX_THROTTLE))) {
            digitalWrite(PWM_1_PIN, HIGH);
        } else {
            digitalWrite(PWM_1_PIN, LOW);
        }
    } else {
        digitalWrite(PWM_1_PIN, LOW);
    }
}
