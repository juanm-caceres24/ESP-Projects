#include <Arduino.h>
#include <Preferences.h>
#include "driver/pcnt.h"

// Encoder pins and settings.
#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define PCNT_UNIT_USED PCNT_UNIT_0
#define WHEEL_MAX_POSITION 6750 // Max position in [encoder counts]

// HALL sensors pins and settings.
#define HALL_ACCEL_PIN 14
#define HALL_BRAKE_PIN 13
#define CALIB_SAMPLES 50
#define CALIB_DELAY_BETWEEN_SAMPLES 10 // Delay in [ms]

// UART pins and settings for secondary controller inputs (buttons/extra controls).
#define RXD1 18
#define TXD1 17
#define UART_BAUD 115200
#define START_BYTE 0xAA
#define PACKET_SIZE 6 // Start byte (1) + Type (1) + ID (1) + Value (2) + CRC (1)
#define MSG_TYPE_BUTTON 0x01
#define MSG_TYPE_ENCODER 0x02 // -NOT CURRENTLY USED-
#define MSG_TYPE_ANALOG 0x03 // -NOT CURRENTLY USED-

// Matrix keypad pins and settings.
#define KEYPAD_ROW_0 1
#define KEYPAD_ROW_1 2
#define KEYPAD_ROW_2 47
#define KEYPAD_ROW_3 21
#define KEYPAD_COL_0 11
#define KEYPAD_COL_1 10
#define KEYPAD_COL_2 9
#define KEYPAD_COL_3 8

// USB CDC bridge packet settings (ESP <-> Python app).
#define BRIDGE_START_BYTE 0xAB
#define BRIDGE_PKT_TELEMETRY 0x10
#define BRIDGE_PKT_TORQUE_CMD 0x20
#define BRIDGE_PKT_CONTROL_CMD 0x30
#define BRIDGE_TELEMETRY_INTERVAL_MS 5

// Bridge command IDs.
#define BRIDGE_CMD_STARTUP 0x01

// Startup flags bitmask (value field of BRIDGE_CMD_STARTUP).
#define STARTUP_FLAG_AUTO_CALIB 0x0001
#define STARTUP_FLAG_SKIP_SELF_TEST 0x0002

// Driver BTS7960B pins and settings.
#define BTS_RPWM_PIN 6
#define BTS_LPWM_PIN 7
#define BTS_REN_PIN 15
#define BTS_LEN_PIN 16
#define BTS_PWM_CH_R 0 // PWM channel for right direction.
#define BTS_PWM_CH_L 1 // PWM channel for left direction.
#define BTS_PWM_FREQ 20000 // Frequency in [Hz]
#define BTS_PWM_RES_BITS 10 // Resolution in [bits]
#define BTS_PWM_MAX ((1 << BTS_PWM_RES_BITS) - 1)

// Force Feedback (FFB) settings.
#define FFB_ABS_MAX_VAL 1000
#define FFB_DEADZONE 5

// Force Feedback PID settings.
#define FFB_POS_KP 0.28f
#define FFB_POS_KI 0.6f
#define FFB_POS_KD 0.0f
#define FFB_POS_I_LIMIT 270.0f

// Position PID controller settings for the motor self-test.
#define MOTOR_SELF_TEST_TORQUE_LIMIT 300
#define MOTOR_SELF_TEST_POS_TOL 60
#define MOTOR_SELF_TEST_HOLD_TIME_MS 500
#define MOTOR_SELF_TEST_TIMEOUT_MS 5000
#define MOTOR_SELF_TEST_POSITION 2250 // Position in [encoder counts] to move during self-test.

// Debug settings.
#define DEBUG_TEXT_LOG 0
#define DEBUG_TELEMETRY_INTERVAL_MS 200

// Button bit layout sent to the PC.
#define KEYPAD_BUTTON_COUNT 16
#define SECONDARY_BUTTON_OFFSET 16
#define SECONDARY_BUTTON_COUNT 16
#define KEYPAD_DEBOUNCE_MS 20

// Telemetry variables.
int16_t joyX = 0;
int16_t joyY = 0;
int16_t joyZ = 0;
uint32_t keypad_buttons = 0;
uint32_t secondary_buttons = 0;
uint32_t hid_buttons = 0;

// UART variables for secondary controller.
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;

// Keypad variables.
const uint8_t keypad_row_pins[4] = {KEYPAD_ROW_0, KEYPAD_ROW_1, KEYPAD_ROW_2, KEYPAD_ROW_3};
const uint8_t keypad_col_pins[4] = {KEYPAD_COL_0, KEYPAD_COL_1, KEYPAD_COL_2, KEYPAD_COL_3};
bool keypad_debounced_state[KEYPAD_BUTTON_COUNT] = {false};
bool keypad_last_raw_state[KEYPAD_BUTTON_COUNT] = {false};
uint32_t keypad_last_change_ms[KEYPAD_BUTTON_COUNT] = {0};

// Hall sensor variables.
uint16_t hall_accel_val = 0;
uint16_t hall_accel_min = 0;
uint16_t hall_accel_max = 4095;
uint16_t hall_brake_val = 0;
uint16_t hall_brake_min = 0;
uint16_t hall_brake_max = 4095;

// Bridge RX parser state.
uint8_t bridge_rx_buf[32];
uint8_t bridge_rx_idx = 0;
uint8_t bridge_rx_expected = 0;

// Force feedback (FFB) variables.
volatile int16_t ffb_torque_value = 0;
volatile uint8_t ffb_device_gain = 255;

// PID controller variables.
int16_t ffb_position = 0;
int32_t ffb_last_position = 0;
int32_t ffb_last_position_error = 0;
int32_t ffb_position_integral = 0;
float ffb_position_derivative = 0.0f;
uint32_t ffb_position_time_us = 0;
uint32_t ffb_last_position_time_us = 0;

// Misc timers.
uint32_t last_debug_ms = 0;
uint32_t last_bridge_telem_ms = 0;

// Startup mode state (controlled by bridge command).
bool bridge_startup_received = false;
uint16_t bridge_startup_flags = 0;

#if DEBUG_TEXT_LOG
#define LOGF(...) Serial.printf(__VA_ARGS__)
#define LOG(...) Serial.println(__VA_ARGS__)
#else
#define LOGF(...)
#define LOG(...)
#endif

/*
 * SENSORS INITIALIZATIONS
 */

void encoder_init() {
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
    pcnt_counter_pause(PCNT_UNIT_USED);
    pcnt_counter_clear(PCNT_UNIT_USED);
    pcnt_counter_resume(PCNT_UNIT_USED);
}

void hall_init() {
    analogReadResolution(12);
    analogSetPinAttenuation(HALL_ACCEL_PIN, ADC_11db);
    analogSetPinAttenuation(HALL_BRAKE_PIN, ADC_11db);
    pinMode(HALL_ACCEL_PIN, INPUT);
    pinMode(HALL_BRAKE_PIN, INPUT);
}

bool hall_calib_values_valid(uint16_t accel_min, uint16_t accel_max, uint16_t brake_min, uint16_t brake_max) {
    if (accel_min >= accel_max || brake_min >= brake_max) {
        return false;
    }
    if (accel_min > 4095 || accel_max > 4095 || brake_min > 4095 || brake_max > 4095) {
        return false;
    }
    return true;
}

bool load_hall_calib_from_flash() {
    Preferences prefs;
    if (!prefs.begin("pedal_calib", true)) {
        return false;
    }
    const bool valid = prefs.getBool("valid", false);
    if (!valid) {
        prefs.end();
        return false;
    }
    const uint16_t accel_min = prefs.getUShort("acc_min", 0);
    const uint16_t accel_max = prefs.getUShort("acc_max", 4095);
    const uint16_t brake_min = prefs.getUShort("brk_min", 0);
    const uint16_t brake_max = prefs.getUShort("brk_max", 4095);
    prefs.end();
    if (!hall_calib_values_valid(accel_min, accel_max, brake_min, brake_max)) {
        return false;
    }
    hall_accel_min = accel_min;
    hall_accel_max = accel_max;
    hall_brake_min = brake_min;
    hall_brake_max = brake_max;
    return true;
}

bool save_hall_calib_to_flash() {
    if (!hall_calib_values_valid(hall_accel_min, hall_accel_max, hall_brake_min, hall_brake_max)) {
        return false;
    }
    Preferences prefs;
    if (!prefs.begin("pedal_calib", false)) {
        return false;
    }
    prefs.putUShort("acc_min", hall_accel_min);
    prefs.putUShort("acc_max", hall_accel_max);
    prefs.putUShort("brk_min", hall_brake_min);
    prefs.putUShort("brk_max", hall_brake_max);
    prefs.putBool("valid", true);
    prefs.end();
    return true;
}

void keypad_init() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(keypad_row_pins[i], OUTPUT);
        digitalWrite(keypad_row_pins[i], HIGH);
        pinMode(keypad_col_pins[i], INPUT_PULLUP);
    }
}

uint32_t scan_keypad_raw_mask() {
    uint32_t mask = 0;
    for (uint8_t row = 0; row < 4; row++) {
        digitalWrite(keypad_row_pins[row], LOW);
        delayMicroseconds(3);
        for (uint8_t col = 0; col < 4; col++) {
            if (digitalRead(keypad_col_pins[col]) == LOW) {
                mask |= (1u << (row * 4 + col));
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
            keypad_debounced_state[i] = raw_pressed;
        }
        if (keypad_debounced_state[i]) {
            keypad_buttons |= (1u << i);
        } else {
            keypad_buttons &= ~(1u << i);
        }
    }
}

/*
 * POSITION VALUES GETTERS
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
    LOG("Calibrating...");
    hall_accel_min = 0;
    hall_brake_min = 0;
    hall_accel_max = 4095;
    hall_brake_max = 4095;
    delay(4000);
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        hall_accel_val = analogRead(HALL_ACCEL_PIN);
        if (hall_accel_val > hall_accel_min) {
            hall_accel_min = hall_accel_val;
        }
        hall_brake_val = analogRead(HALL_BRAKE_PIN);
        if (hall_brake_val > hall_brake_min) {
            hall_brake_min = hall_brake_val;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }
    delay(4000);
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        hall_accel_val = analogRead(HALL_ACCEL_PIN);
        if (hall_accel_val < hall_accel_max) {
            hall_accel_max = hall_accel_val;
        }
        hall_brake_val = analogRead(HALL_BRAKE_PIN);
        if (hall_brake_val < hall_brake_max) {
            hall_brake_max = hall_brake_val;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }
    if (save_hall_calib_to_flash()) {
        LOG("Calibration done. Values saved to flash.");
    } else {
        LOG("Calibration done, but saving to flash failed.");
    }
}

/*
 * SECONDARY UART PACKET HANDLING (buttons)
 */

uint8_t calculate_crc_xor(const uint8_t* data, uint8_t len_without_crc) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len_without_crc; i++) {
        crc ^= data[i];
    }
    return crc;
}

void process_secondary_packet(uint8_t *packet) {
    uint8_t type = packet[1];
    uint8_t id = packet[2];
    uint16_t value = packet[3] | (packet[4] << 8);
    switch (type) {
        case MSG_TYPE_BUTTON:
            if (id < SECONDARY_BUTTON_COUNT) {
                const uint8_t bit_index = SECONDARY_BUTTON_OFFSET + id;
                if (value) {
                    secondary_buttons |= (1u << bit_index);
                } else {
                    secondary_buttons &= ~(1u << bit_index);
                }
            }
            break;
        case MSG_TYPE_ENCODER:
            break;
        case MSG_TYPE_ANALOG:
            break;
        default:
            break;
    }
}

void handle_secondary_uart() {
    while (Serial1.available()) {
        uint8_t byte_received = Serial1.read();
        if (rxIndex == 0) {
            if (byte_received == START_BYTE) {
                rxBuffer[rxIndex++] = byte_received;
            }
        } else {
            rxBuffer[rxIndex++] = byte_received;
            if (rxIndex >= PACKET_SIZE) {
                uint8_t received_crc = rxBuffer[5];
                uint8_t calculated_crc = calculate_crc_xor(rxBuffer, 5);
                if (received_crc == calculated_crc) {
                    process_secondary_packet(rxBuffer);
                }
                rxIndex = 0;
            }
        }
    }
}

/*
 * BRIDGE PACKET HANDLING (USB CDC serial)
 */

uint8_t bridge_packet_len(uint8_t type) {
    if (type == BRIDGE_PKT_TORQUE_CMD) {
        return 8;
    }
    if (type == BRIDGE_PKT_CONTROL_CMD) {
        return 7;
    }
    return 0;
}

void process_bridge_command(uint8_t cmd, int16_t value) {
    switch (cmd) {
        case BRIDGE_CMD_STARTUP:
            bridge_startup_flags = (uint16_t)value;
            bridge_startup_received = true;
            break;
        default:
            break;
    }
}

void process_bridge_packet(const uint8_t* pkt, uint8_t len) {
    if (len < 4) {
        return;
    }
    const uint8_t type = pkt[1];
    if (type == BRIDGE_PKT_TORQUE_CMD && len >= 8) {
        int16_t torque = (int16_t)((uint16_t)pkt[3] | ((uint16_t)pkt[4] << 8));
        uint8_t gain = pkt[5];
        ffb_torque_value = constrain(torque, -FFB_ABS_MAX_VAL, FFB_ABS_MAX_VAL);
        ffb_device_gain = gain;
        return;
    }
    if (type == BRIDGE_PKT_CONTROL_CMD && len >= 7) {
        const uint8_t cmd = pkt[3];
        const int16_t value = (int16_t)((uint16_t)pkt[4] | ((uint16_t)pkt[5] << 8));
        process_bridge_command(cmd, value);
    }
}

void handle_bridge_serial() {
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        if (bridge_rx_idx == 0) {
            if (b == BRIDGE_START_BYTE) {
                bridge_rx_buf[bridge_rx_idx++] = b;
                bridge_rx_expected = 0;
            }
            continue;
        }
        if (bridge_rx_idx >= sizeof(bridge_rx_buf)) {
            bridge_rx_idx = 0;
            bridge_rx_expected = 0;
            continue;
        }
        bridge_rx_buf[bridge_rx_idx++] = b;
        if (bridge_rx_idx == 2) {
            bridge_rx_expected = bridge_packet_len(bridge_rx_buf[1]);
            if (bridge_rx_expected == 0 || bridge_rx_expected > sizeof(bridge_rx_buf)) {
                bridge_rx_idx = 0;
                bridge_rx_expected = 0;
            }
            continue;
        }
        if (bridge_rx_expected > 0 && bridge_rx_idx >= bridge_rx_expected) {
            uint8_t crc_recv = bridge_rx_buf[bridge_rx_expected - 1];
            uint8_t crc_calc = calculate_crc_xor(bridge_rx_buf, bridge_rx_expected - 1);
            if (crc_recv == crc_calc) {
                process_bridge_packet(bridge_rx_buf, bridge_rx_expected);
            }
            bridge_rx_idx = 0;
            bridge_rx_expected = 0;
        }
    }
}

void send_bridge_telemetry() {
    uint8_t pkt[14];
    pkt[0] = BRIDGE_START_BYTE;
    pkt[1] = BRIDGE_PKT_TELEMETRY;
    pkt[2] = 0; // Reserved for sequence number.
    pkt[3] = (uint8_t)(joyX & 0xFF);
    pkt[4] = (uint8_t)((joyX >> 8) & 0xFF);
    pkt[5] = (uint8_t)(joyY & 0xFF);
    pkt[6] = (uint8_t)((joyY >> 8) & 0xFF);
    pkt[7] = (uint8_t)(joyZ & 0xFF);
    pkt[8] = (uint8_t)((joyZ >> 8) & 0xFF);
    pkt[9] = (uint8_t)(hid_buttons & 0xFF);
    pkt[10] = (uint8_t)((hid_buttons >> 8) & 0xFF);
    pkt[11] = (uint8_t)((hid_buttons >> 16) & 0xFF);
    pkt[12] = (uint8_t)((hid_buttons >> 24) & 0xFF);
    pkt[13] = calculate_crc_xor(pkt, 13);
    Serial.write(pkt, sizeof(pkt));
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
    return constrain(FFB_POS_KP * ffb_position_error + FFB_POS_KI * ffb_position_integral + FFB_POS_KD * ffb_position_derivative, -FFB_ABS_MAX_VAL, FFB_ABS_MAX_VAL);
}

void reset_position_pid_state() {
    ffb_position_integral = 0;
    ffb_last_position_error = 0;
    ffb_position_derivative = 0.0f;
}

/*
 * FORCE FEEDBACK - MOTOR CONTROL
 */

void motor_init() {
    pinMode(BTS_REN_PIN, OUTPUT);
    pinMode(BTS_LEN_PIN, OUTPUT);
    digitalWrite(BTS_REN_PIN, HIGH);
    digitalWrite(BTS_LEN_PIN, HIGH);
    ledcSetup(BTS_PWM_CH_R, BTS_PWM_FREQ, BTS_PWM_RES_BITS);
    ledcSetup(BTS_PWM_CH_L, BTS_PWM_FREQ, BTS_PWM_RES_BITS);
    ledcAttachPin(BTS_RPWM_PIN, BTS_PWM_CH_R);
    ledcAttachPin(BTS_LPWM_PIN, BTS_PWM_CH_L);
    ledcWrite(BTS_PWM_CH_R, 0);
    ledcWrite(BTS_PWM_CH_L, 0);
}

void set_motor_torque(int16_t torque_val) {
    torque_val = constrain(torque_val, -FFB_ABS_MAX_VAL, FFB_ABS_MAX_VAL);
    int32_t scaled = (int32_t)torque_val * ffb_device_gain / 255;
    int pwm = map(abs((int)scaled), 0, FFB_ABS_MAX_VAL, 0, BTS_PWM_MAX);
    if (pwm < FFB_DEADZONE) {
        pwm = 0;
    }
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
    uint32_t start_time = millis();
    uint32_t hold_position_time = 0;
    uint8_t last_holding_position_flag = false;
    int16_t target_position = -MOTOR_SELF_TEST_POSITION;
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
                LOG("Minimum position reached.");
                break;
            }
        } else {
            last_holding_position_flag = false;
        }
        if (millis() - start_time > MOTOR_SELF_TEST_TIMEOUT_MS) {
            LOG("Motor self-test failed: timeout reached.");
            break;
        }
        delay(10);
    }
    start_time = millis();
    hold_position_time = 0;
    last_holding_position_flag = false;
    target_position = MOTOR_SELF_TEST_POSITION;
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
                LOG("Maximum position reached.");
                break;
            }
        } else {
            last_holding_position_flag = false;
        }
        if (millis() - start_time > MOTOR_SELF_TEST_TIMEOUT_MS) {
            LOG("Motor self-test failed: timeout reached.");
            break;
        }
        delay(10);
    }
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
                LOG("Returned to center position.");
                break;
            }
        } else {
            last_holding_position_flag = false;
        }
        if (millis() - start_time > MOTOR_SELF_TEST_TIMEOUT_MS) {
            LOG("Motor self-test failed: timeout reached.");
            break;
        }
        delay(10);
    }
}

void setup() {
    // USB CDC serial is the host interface used to receive startup command and runtime packets.
    Serial.begin(115200);
    // Wait until the bridge sends a startup command.
    while (!bridge_startup_received) {
        handle_bridge_serial();
        delay(1);
    }
    // Normal peripheral initialization only after startup command.
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1, TXD1);
    keypad_init();
    encoder_init();
    hall_init();
    motor_init();
    const bool auto_calib = (bridge_startup_flags & STARTUP_FLAG_AUTO_CALIB) != 0;
    const bool skip_self_test = (bridge_startup_flags & STARTUP_FLAG_SKIP_SELF_TEST) != 0;
    if (auto_calib) {
        hall_calib();
    } else if (!load_hall_calib_from_flash()) {
        LOG("No valid pedal calibration in flash. Using default full ADC range.");
    }

    if (!skip_self_test) {
        motor_self_test();
    }

    pcnt_get_counter_value(PCNT_UNIT_USED, &ffb_position);
}

void loop() {
    update_keypad_buttons();
    handle_secondary_uart();
    hid_buttons = keypad_buttons | secondary_buttons;
    handle_bridge_serial();
    update_position();
    joyX = get_wheel_position();
    joyY = get_accel_position();
    joyZ = get_brake_position();
    uint32_t now = millis();
    set_motor_torque(ffb_torque_value);
    if ((uint32_t)(now - last_bridge_telem_ms) >= BRIDGE_TELEMETRY_INTERVAL_MS) {
        last_bridge_telem_ms = now;
        send_bridge_telemetry();
    }
    if ((uint32_t)(now - last_debug_ms) >= DEBUG_TELEMETRY_INTERVAL_MS) {
        last_debug_ms = now;
        LOGF("X:%6d raw:%6d Y:%6d Z:%6d torque:%5d gain:%3d\n", joyX, ffb_position, joyY, joyZ, ffb_torque_value, ffb_device_gain);
    }
    delay(1);
}
