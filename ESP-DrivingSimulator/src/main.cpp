#include <Arduino.h>
#include "driver/pcnt.h"
#include "USB.h"
#include "USBHIDGamepad16.h"

// Encoder pins and settings.
#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define PCNT_UNIT_USED PCNT_UNIT_0
#define WHEEL_MAX_POSITION 6750 // Max position in [encoder counts]

// HALL sensors pins and settings
#define HALL_ACCEL_PIN 14
#define HALL_BRAKE_PIN 13
#define CALIB_SAMPLES 50
#define CALIB_DELAY_BETWEEN_SAMPLES 10 // Delay in [ms]

// UART pins and settings.
#define RXD1 18
#define TXD1 17
#define UART_BAUD 115200 // Baud rate for UART communication.
#define START_BYTE 0xAA // Arbitrary start byte to indicate the beginning of a packet.
#define PACKET_SIZE 6 // Start byte (1) + Type (1) + ID (1) + Value (2) + CRC (1)
#define MSG_TYPE_BUTTON 0x01
#define MSG_TYPE_ENCODER 0x02 // -NOT CURRENTLY USED-
#define MSG_TYPE_ANALOG 0x03 // -NOT CURRENTLY USED-

// Driver BTS7960B pins and settings.
#define BTS_RPWM_PIN 6
#define BTS_LPWM_PIN 7
#define BTS_REN_PIN 15
#define BTS_LEN_PIN 16
#define BTS_PWM_CH_R 0 // PWM channel for right direction.
#define BTS_PWM_CH_L 1 // PWM channel for left direction.
#define BTS_PWM_FREQ 20000 // Frequency in [Hz]
#define BTS_PWM_RES_BITS 10 // Resolution in [bits] indicates the quantity of discrete steps in the PWM signal. For example, 10 bits means 1024 steps (0-1023).
#define BTS_PWM_MAX ((1 << BTS_PWM_RES_BITS) - 1) // Maximum PWM value based on resolution bits.

// Force Feedback (FFB) settings.
#define FFB_MAX_VAL 1024 // Absolute maximum FFB value (full torque).
#define FFB_DEADZONE 5 // Command values with absolute value below this will be considered zero to prevent motor from trying to compensate for small non-zero values.
#define FFB_RID_VENDOR_TORQUE 0x20 // Report ID for vendor-specific torque commands. This is an arbitrary value chosen for this project.

// Force Feedback PID settings.
#define FFB_POS_KP 0.28f // Proportional gain for position-based FFB.
#define FFB_POS_KI 0.6f // Integral gain for position-based FFB.
#define FFB_POS_KD 0.0f // Derivative gain for position-based FFB.
#define FFB_POS_I_LIMIT 270.0f // Integral windup limit for position-based FFB.

// Position PID controller settings for the motor self-test.
#define MOTOR_SELF_TEST_TORQUE_LIMIT 300 // Maximum torque value during the self-test.
#define MOTOR_SELF_TEST_POS_TOL 400 // Position tolerance in [encoder counts] for the self-test to consider the position reached.
#define MOTOR_SELF_TEST_HOLD_TIME_MS 500 // Time in [ms] to hold the position during the self-test once it's reached.
#define MOTOR_SELF_TEST_TIMEOUT_MS 5000 // Timeout for the self-test in milliseconds.

// Debug settings.
#define DEBUG_TELEMETRY_INTERVAL_MS 200 // Interval in [ms] for printing debug telemetry to the serial console.

// Gamepad variables.
USBHIDGamepad16 gamepad;
int16_t joyX;
int16_t joyY;
int16_t joyZ;

// UART variables.
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
uint32_t hid_buttons = 0;
uint32_t last_debug_ms = 0; // Timestamp of the last debug telemetry print in [ms].

// Hall sensor variables to store the latest readings and calibration values.
uint16_t hall_accel_val = 0;
uint16_t hall_accel_min = 0;
uint16_t hall_accel_max = 4095;
uint16_t hall_brake_val = 0;
uint16_t hall_brake_min = 0;
uint16_t hall_brake_max = 4095;

// Force feedback (FFB) variables.
volatile int16_t ffb_torque_value = 0;
volatile uint8_t ffb_device_gain = 255;
volatile uint32_t ffb_last_cmd_ms = 0; // Timestamp of the last received FFB command, used for timeout handling.

// PID controller variables for FFB calculations.
// Positions, velocities and accelerations.
int16_t ffb_position = 0; // Current position in [encoder counts].
int32_t ffb_last_position = 0; // Last position in [encoder counts] used for velocity calculation.
// PID errors.
int32_t ffb_position_error = 0;
int32_t ffb_last_position_error = 0;
// PID integrals and derivatives.
int32_t ffb_position_integral = 0;
float ffb_position_derivative = 0.0f;
// Timestamps of the last PID calculations for each control loop.
uint32_t ffb_position_time_us = 0; // Timestamp of the current position calculation in [us].
uint32_t ffb_last_position_time_us = 0; // Timestamp of the last position calculation in [us].

/*
 * SENSORS INITIALIZATIONS
 */

void encoder_init() {
    // x4 quadrature decoding: count edges from both channels (A and B).
    pcnt_config_t pcnt_config_ch0 = {};
    pcnt_config_ch0.pulse_gpio_num = ENCODER_PIN_A;
    pcnt_config_ch0.ctrl_gpio_num = ENCODER_PIN_B;
    pcnt_config_ch0.channel = PCNT_CHANNEL_0;
    pcnt_config_ch0.unit = PCNT_UNIT_USED;
    pcnt_config_ch0.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_ch0.neg_mode = PCNT_COUNT_INC;
    pcnt_config_ch0.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_ch0.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_ch0.counter_h_lim = 32767;
    pcnt_config_ch0.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config_ch0);
    pcnt_config_t pcnt_config_ch1 = {};
    pcnt_config_ch1.pulse_gpio_num = ENCODER_PIN_B;
    pcnt_config_ch1.ctrl_gpio_num = ENCODER_PIN_A;
    pcnt_config_ch1.channel = PCNT_CHANNEL_1;
    pcnt_config_ch1.unit = PCNT_UNIT_USED;
    pcnt_config_ch1.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_ch1.neg_mode = PCNT_COUNT_INC;
    pcnt_config_ch1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_ch1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_ch1.counter_h_lim = 32767;
    pcnt_config_ch1.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config_ch1);
    pcnt_counter_pause(PCNT_UNIT_USED); // Start with counter paused while we set it up.
    pcnt_counter_clear(PCNT_UNIT_USED); // Clear counter to start at zero.
    pcnt_counter_resume(PCNT_UNIT_USED); // Resume counting.
}

void hall_init() {
    analogReadResolution(12);
    analogSetPinAttenuation(HALL_ACCEL_PIN, ADC_11db);
    analogSetPinAttenuation(HALL_BRAKE_PIN, ADC_11db);
    pinMode(HALL_ACCEL_PIN, INPUT);
    pinMode(HALL_BRAKE_PIN, INPUT);
}

/*
 * POSITION VALUES GETTERS FOR HID REPORTS
 */

int16_t get_wheel_position() {
    int16_t position = 0;
    pcnt_get_counter_value(PCNT_UNIT_USED, &position);
    position = constrain(position, -WHEEL_MAX_POSITION, WHEEL_MAX_POSITION);
    return map(position, -WHEEL_MAX_POSITION, WHEEL_MAX_POSITION, -32768, 32767);
}

int16_t get_accel_position() {
    uint16_t position = analogRead(HALL_ACCEL_PIN);
    position = constrain(position, hall_accel_min, hall_accel_max);
    return map(position, hall_accel_min, hall_accel_max, -32768, 32767);
}

int16_t get_brake_position() {
    uint16_t position = analogRead(HALL_BRAKE_PIN);
    position = constrain(position, hall_brake_min, hall_brake_max);
    return map(position, hall_brake_min, hall_brake_max, -32768, 32767);
}

void update_position() {
    ffb_last_position = ffb_position;
    ffb_last_position_time_us = ffb_position_time_us;
    ffb_position_time_us = micros();
    pcnt_get_counter_value(PCNT_UNIT_USED, &ffb_position);
}

/*
 * HALL SENSORS CALIBRATION
 */

void hall_calib() {
    Serial.printf("Calibrating...\n");
    Serial.printf("Please leave the pedals untouched in 3 seconds...\n");
    delay(4000);
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        hall_accel_val = analogRead(HALL_ACCEL_PIN);
        // Check the maximum value reached in the minimum position and assume that's the actual minimum.
        if (hall_accel_val > hall_accel_min) {
            hall_accel_min = hall_accel_val;
        }
        hall_brake_val = analogRead(HALL_BRAKE_PIN);
        // Check the maximum value reached in the minimum position and assume that's the actual minimum.
        if (hall_brake_val > hall_brake_min) {
            hall_brake_min = hall_brake_val;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }
    Serial.printf("Please fully press the pedals and hold for 3 seconds...\n");
    delay(4000);
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        hall_accel_val = analogRead(HALL_ACCEL_PIN);
        // Check the minimum value reached in the maximum position and assume that's the actual maximum.
        if (hall_accel_val < hall_accel_max) {
            hall_accel_max = hall_accel_val;
        }
        hall_brake_val = analogRead(HALL_BRAKE_PIN);
        // Check the minimum value reached in the maximum position and assume that's the actual maximum.
        if (hall_brake_val < hall_brake_max) {
            hall_brake_max = hall_brake_val;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }
    Serial.printf("Calibration done.\n");
}

/*
 * UART PACKET HANDLING
 */

uint8_t calculate_CRC(uint8_t *data) {
    uint8_t crc = 0;
    for (int i = 0; i < 5; i++) {
        crc ^= data[i];
    }
    return crc;
}

void process_packet(uint8_t *packet) {
    uint8_t type = packet[1];
    uint8_t id = packet[2];
    uint16_t value = packet[3] | (packet[4] << 8);
    switch (type) {
        case MSG_TYPE_BUTTON:
            if (id == 0) {
                if (value) {
                    hid_buttons |= (1u << BUTTON_A);
                } else {
                    hid_buttons &= ~(1u << BUTTON_A);
                }
            }
            if (id == 1) {
                if (value) {
                    hid_buttons |= (1u << BUTTON_B);
                } else {
                    hid_buttons &= ~(1u << BUTTON_B);
                }
            }
            break;
        case MSG_TYPE_ENCODER: // -NOT CURRENTLY USED-
            break;
        case MSG_TYPE_ANALOG: // -NOT CURRENTLY USED-
            break;
        default:
            break;
    }
}

void handle_UART() {
    while (Serial1.available()) {
        uint8_t byteReceived = Serial1.read();
        if (rxIndex == 0) {
            if (byteReceived == START_BYTE) {
                rxBuffer[rxIndex++] = byteReceived;
            }
        } else {
            rxBuffer[rxIndex++] = byteReceived;
            if (rxIndex >= PACKET_SIZE) {
                uint8_t receivedCRC = rxBuffer[5];
                uint8_t calculatedCRC = calculate_CRC(rxBuffer);
                if (receivedCRC == calculatedCRC) {
                    process_packet(rxBuffer);
                }
                rxIndex = 0;
            }
        }
    }
}

/*
 * PID CONTROL METHODS
 */

int16_t get_pos_PID(int32_t target_position) {
    uint32_t dt_us = (ffb_position_time_us - ffb_last_position_time_us);
    int32_t ffb_position_error = target_position - ffb_position;
    if (dt_us > 0) {
        ffb_position_integral += ffb_position_error * dt_us;
        ffb_position_integral = constrain(ffb_position_integral, -FFB_POS_I_LIMIT, FFB_POS_I_LIMIT);
        ffb_position_derivative = (float)(ffb_position_error - ffb_last_position_error) / (float)dt_us;
    } else {
        ffb_position_derivative = 0.0f;
    }
    ffb_last_position_error = ffb_position_error;
    return constrain(FFB_POS_KP * ffb_position_error + FFB_POS_KI * ffb_position_integral + FFB_POS_KD * ffb_position_derivative, -FFB_MAX_VAL, FFB_MAX_VAL);
}

void reset_position_pid_state() {
    ffb_position_integral = 0;
    ffb_last_position_error = 0;
    ffb_position_derivative = 0.0f;
}

/*
 * FORCE FEEDBACK
 */

void motor_init() {
    // Configure the BTS7960B motor driver pins.
    pinMode(BTS_REN_PIN, OUTPUT);
    pinMode(BTS_LEN_PIN, OUTPUT);
    // Set both enable pins high to allow the motor to be driven in both directions. The actual direction and torque will be controlled by the PWM signals on the RPWM and LPWM pins.
    digitalWrite(BTS_REN_PIN, HIGH);
    digitalWrite(BTS_LEN_PIN, HIGH);
    // Configure PWM channels for both directions and attach them to the corresponding pins.
    ledcSetup(BTS_PWM_CH_R, BTS_PWM_FREQ, BTS_PWM_RES_BITS);
    ledcSetup(BTS_PWM_CH_L, BTS_PWM_FREQ, BTS_PWM_RES_BITS);
    // Start with 0 torque (0% duty cycle) for both directions.
    ledcAttachPin(BTS_RPWM_PIN, BTS_PWM_CH_R);
    ledcAttachPin(BTS_LPWM_PIN, BTS_PWM_CH_L);
    // Ensure motor is stopped at initialization.
    ledcWrite(BTS_PWM_CH_R, 0);
    ledcWrite(BTS_PWM_CH_L, 0);
}

void set_motor_torque(int16_t torque_val) {
    torque_val = constrain(torque_val, -FFB_MAX_VAL, FFB_MAX_VAL);
    // Scale the FFB value by the device gain (0-255) to get the actual torque value to be applied.
    int32_t scaled = (int32_t)torque_val * ffb_device_gain / 255;
    // Map the absolute value of the scaled FFB value to a PWM value
    int pwm = map(abs((int)scaled), 0, FFB_MAX_VAL, 0, BTS_PWM_MAX);
    // Apply deadzone to prevent motor from trying to compensate for small non-zero values that could be caused by noise or minor command fluctuations.
    if (pwm < FFB_DEADZONE) {
        pwm = 0;
    }
    // Set PWM values for the corresponding direction based on the sign of the scaled command.
    if (scaled > 0) {
        ledcWrite(BTS_PWM_CH_R, pwm);
        ledcWrite(BTS_PWM_CH_L, 0);
    } else if (scaled < 0) {
        ledcWrite(BTS_PWM_CH_R, 0);
        ledcWrite(BTS_PWM_CH_L, pwm);
    } else {
        ledcWrite(BTS_PWM_CH_R, 0);
        ledcWrite(BTS_PWM_CH_L, 0);
    }
}

void motor_self_test() {
    // Loop until we reach the min target position within the tolerance and hold it for the specified time, or until we hit the timeout.
    uint32_t start_time = millis();
    uint32_t hold_position_time = 0; // Time to hold the position once it's reached, to ensure the motor can maintain it and it's not just briefly passing through it.
    uint8_t last_holding_position_flag = false; // Flag to indicate if we are currently holding the position within the tolerance.
    int16_t target_position = -WHEEL_MAX_POSITION;
    reset_position_pid_state();
    while (true) {
        update_position();
        float pid_output = get_pos_PID(target_position);
        int16_t torque_command = (int16_t)constrain(pid_output, -MOTOR_SELF_TEST_TORQUE_LIMIT, MOTOR_SELF_TEST_TORQUE_LIMIT);
        set_motor_torque(torque_command);
        int16_t position_error = target_position - ffb_position;
        if (abs(position_error) <= MOTOR_SELF_TEST_POS_TOL) {
            if (!last_holding_position_flag) {
                hold_position_time = millis();
                last_holding_position_flag = true;
            } else if (millis() - hold_position_time >= MOTOR_SELF_TEST_HOLD_TIME_MS) {
                Serial.println("Minimum position reached.");
                break;
            }
        } else {
            last_holding_position_flag = false;
        }
        // If the timeout is reached, stop the motor and exit the test.
        if (millis() - start_time > MOTOR_SELF_TEST_TIMEOUT_MS) {
            Serial.println("Motor self-test failed: timeout reached.");
            break;
        }
        delay(10);
    }
    // Loop until we reach the max target position WHEEL_MAX_POSITION within the tolerance and hold it for the specified time, or until we hit the timeout.
    start_time = millis();
    hold_position_time = 0;
    last_holding_position_flag = false;
    target_position = WHEEL_MAX_POSITION;
    reset_position_pid_state();
    while (true) {
        update_position();
        float pid_output = get_pos_PID(target_position);
        int16_t torque_command = (int16_t)constrain(pid_output, -MOTOR_SELF_TEST_TORQUE_LIMIT, MOTOR_SELF_TEST_TORQUE_LIMIT);
        set_motor_torque(torque_command);
        int16_t position_error = target_position - ffb_position;
        if (abs(position_error) <= MOTOR_SELF_TEST_POS_TOL) {
            if (!last_holding_position_flag) {
                hold_position_time = millis();
                last_holding_position_flag = true;
            } else if (millis() - hold_position_time >= MOTOR_SELF_TEST_HOLD_TIME_MS) {
                Serial.println("Maximum position reached.");
                break;
            }
        } else {
            last_holding_position_flag = false;
        }
        // If the timeout is reached, stop the motor and exit the test.
        if (millis() - start_time > MOTOR_SELF_TEST_TIMEOUT_MS) {
            Serial.println("Motor self-test failed: timeout reached.");
            break;
        }
        delay(10);
    }
    // Loop until we return to the center position (0) within the tolerance and hold it for the specified time, to ensure the motor can return to the neutral position, or until we hit the timeout.
    start_time = millis();
    target_position = 0;
    hold_position_time = 0;
    last_holding_position_flag = false;
    reset_position_pid_state();
    while (true) {
        update_position();
        float pid_output = get_pos_PID(target_position);
        int16_t torque_command = (int16_t)constrain(pid_output, -MOTOR_SELF_TEST_TORQUE_LIMIT, MOTOR_SELF_TEST_TORQUE_LIMIT);
        set_motor_torque(torque_command);
        int16_t position_error = target_position - ffb_position;
        if (abs(position_error) <= MOTOR_SELF_TEST_POS_TOL) {
            if (!last_holding_position_flag) {
                hold_position_time = millis();
                last_holding_position_flag = true;
            } else if (millis() - hold_position_time >= MOTOR_SELF_TEST_HOLD_TIME_MS) {
                Serial.println("Returned to center position.");
                break;
            }
        } else {
            last_holding_position_flag = false;
        }
        // If the timeout is reached, stop the motor and exit the test.
        if (millis() - start_time > MOTOR_SELF_TEST_TIMEOUT_MS) {
            Serial.println("Motor self-test failed: timeout reached.");
            break;
        }
        delay(10);
    }
}

/*
 * Callback function for handling HID output reports.
 */

void on_hid_output_report(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    if (report_id != FFB_RID_VENDOR_TORQUE || buffer == nullptr || len < 3) {
        return;
    }
    // Payload format: int16 torque (LE), uint8 gain, then padding.
    int16_t torque = (int16_t)((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8)); // Bits 0-15: torque value in little-endian format. This allows representing values from -32768 to 32767, which we will then constrain to our defined FFB_MAX_VAL range.
    uint8_t gain = buffer[2]; // Bits 16-23: device gain (0-255) to scale the torque value for different devices with different torque capabilities.
    ffb_torque_value = constrain(torque, -FFB_MAX_VAL, FFB_MAX_VAL);
    ffb_device_gain = gain;
    ffb_last_cmd_ms = millis();
}

void setup() {
    // Initialize USB HID gamepad and FFB receiver.
    USB.begin();
    gamepad.setOutputCallback(on_hid_output_report);
    gamepad.begin();
    // Initialize serial communication for debugging.
    Serial.begin(115200);
    // Initialize UART communication for receiving commands from the secondary controller.
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1, TXD1);
    // Initialize encoder.
    encoder_init();
    Serial.println("Encoder sensor ready.");
    // Initialize hall sensors and perform calibration.
    hall_init();
    hall_calib();
    Serial.println("Hall sensors ready.");
    // Initialize motor driver and perform self-test.
    motor_init();
    motor_self_test();
    Serial.println("BTS7960B ready.");
    // Initialize FFB variables.
    pcnt_get_counter_value(PCNT_UNIT_USED, &ffb_position); // Get the initial position from the encoder in [encoder counts].
}

void loop() {
    // Get UART messages and update internal state accordingly.
    handle_UART();
    // Update position, and timestamps.
    update_position();
    // Update motor torque based on the latest FFB value received.
    set_motor_torque(ffb_torque_value);
    // Send HID report with the latest sensor values and button states.
    joyX = get_wheel_position();
    joyY = get_accel_position();
    joyZ = get_brake_position();
    gamepad.send(
        joyX,
        joyY,
        joyZ,
        0,
        0,
        0,
        HAT_CENTER,
        hid_buttons
    );
    // Print debug telemetry to the serial console at a defined interval.
    uint32_t now = millis();
    if (now - last_debug_ms >= DEBUG_TELEMETRY_INTERVAL_MS) {
        last_debug_ms = now;
        Serial.printf("Joystick X: %6d (%6d) Y: %6d Z: %6d FFB value: %5d gain: %3d\n", joyX, ffb_position, joyY, joyZ, ffb_torque_value, ffb_device_gain);
    }
    // Small delay to prevent overwhelming the loop and allow other tasks to run.
    delay(2);
}
