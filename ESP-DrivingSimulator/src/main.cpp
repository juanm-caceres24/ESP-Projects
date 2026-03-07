#include <Arduino.h>
#include "driver/pcnt.h"
#include "USB.h"
#include "USBHIDGamepad16.h"

#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define PCNT_UNIT_USED PCNT_UNIT_0
#define ENCODER_MAX_COUNT 3375

#define HALL_ACCEL_PIN 14
#define HALL_BRAKE_PIN 13
#define CALIB_SAMPLES 50
#define CALIB_DELAY_BETWEEN_SAMPLES 10

#define RXD1 18
#define TXD1 17
#define UART_BAUD 115200
#define START_BYTE 0xAA
#define PACKET_SIZE 6
#define MSG_TYPE_BUTTON 0x01
#define MSG_TYPE_ENCODER 0x02
#define MSG_TYPE_ANALOG 0x03

#define BTS_RPWM_PIN 6
#define BTS_LPWM_PIN 7
#define BTS_REN_PIN 15
#define BTS_LEN_PIN 16

#define BTS_PWM_CH_R 0
#define BTS_PWM_CH_L 1
#define BTS_PWM_FREQ 20000
#define BTS_PWM_RES_BITS 10
#define BTS_PWM_MAX ((1 << BTS_PWM_RES_BITS) - 1)

#define RUN_MOTOR_SELF_TEST_AFTER_CALIB 1
#define MOTOR_SELF_TEST_GAIN 220
#define MOTOR_SELF_TEST_SETTLE_MS 500

// Closed-loop self-test: go to left/right limits and back to center.
#define MOTOR_SELF_TEST_PID_KP 0.015f
#define MOTOR_SELF_TEST_PID_KI 0.04f
#define MOTOR_SELF_TEST_PID_KD 0.0025f
#define MOTOR_SELF_TEST_PID_I_LIMIT 6000.0f
#define MOTOR_SELF_TEST_TORQUE_LIMIT 450
#define MOTOR_SELF_TEST_POS_TOL 40
#define MOTOR_SELF_TEST_VEL_TOL 120
#define MOTOR_SELF_TEST_TIMEOUT_MS 5000
#define MOTOR_SELF_TEST_LOOP_MS 5
#define MOTOR_SELF_TEST_SPAN_RATIO 0.35f
#define MOTOR_SELF_TEST_STALL_POS_DELTA 6
#define MOTOR_SELF_TEST_STALL_TIMEOUT_MS 700

#define FFB_TIMEOUT_MS 2000
#define FFB_MIN_CMD -1000
#define FFB_MAX_CMD 1000
#define FFB_DEADZONE 15
#define FFB_RID_VENDOR_TORQUE 0x20

#define DEBUG_TELEMETRY 1
#define DEBUG_TELEMETRY_INTERVAL_MS 100

struct HallSensor {
    uint8_t pin;
    uint16_t min_value;
    uint16_t max_value;
};

USBHIDGamepad16 gamepad;
HallSensor hall_accel = {HALL_ACCEL_PIN, 0, 4095};
HallSensor hall_brake = {HALL_BRAKE_PIN, 0, 4095};

uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
uint32_t hid_buttons = 0;

volatile int16_t g_ffb_torque_cmd = 0;
volatile uint8_t g_ffb_device_gain = 255;
volatile uint32_t g_last_ffb_ms = 0;
uint32_t g_last_debug_ms = 0;

int16_t encoder_get_count();

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

void set_motor_torque(int16_t torque_cmd) {
    if (millis() - g_last_ffb_ms > FFB_TIMEOUT_MS) {
        ledcWrite(BTS_PWM_CH_R, 0);
        ledcWrite(BTS_PWM_CH_L, 0);
        return;
    }

    torque_cmd = constrain(torque_cmd, FFB_MIN_CMD, FFB_MAX_CMD);
    int32_t scaled = (int32_t)torque_cmd * g_ffb_device_gain / 255;
    int pwm = map(abs((int)scaled), 0, FFB_MAX_CMD, 0, BTS_PWM_MAX);

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

void set_motor_torque_direct(int16_t torque_cmd, uint8_t gain) {
    torque_cmd = constrain(torque_cmd, FFB_MIN_CMD, FFB_MAX_CMD);
    int32_t scaled = (int32_t)torque_cmd * gain / 255;
    int pwm = map(abs((int)scaled), 0, FFB_MAX_CMD, 0, BTS_PWM_MAX);

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

static float clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

bool run_motor_to_target_pid(int16_t target_count, uint8_t gain, uint32_t timeout_ms, uint16_t settle_ms) {
    uint32_t start_ms = millis();
    uint32_t prev_ms = start_ms;
    uint32_t in_tolerance_since = 0;
    uint32_t progress_ms = start_ms;
    int16_t prev_error = target_count - encoder_get_count();
    int16_t prev_pos = encoder_get_count();

    float integral = 0.0f;

    while (millis() - start_ms < timeout_ms) {
        uint32_t now_ms = millis();
        float dt = (now_ms - prev_ms) / 1000.0f;
        if (dt <= 0.0f) {
            dt = MOTOR_SELF_TEST_LOOP_MS / 1000.0f;
        }

        int16_t pos = encoder_get_count();
        int16_t error = target_count - pos;
        float derivative = (error - prev_error) / dt;

        integral += error * dt;
        integral = clampf(integral, -MOTOR_SELF_TEST_PID_I_LIMIT, MOTOR_SELF_TEST_PID_I_LIMIT);

        float cmd =
            (MOTOR_SELF_TEST_PID_KP * error) +
            (MOTOR_SELF_TEST_PID_KI * integral) +
            (MOTOR_SELF_TEST_PID_KD * derivative);

        cmd = clampf(cmd, -MOTOR_SELF_TEST_TORQUE_LIMIT, MOTOR_SELF_TEST_TORQUE_LIMIT);
        set_motor_torque_direct((int16_t)cmd, gain);

        bool pos_ok = abs(error) <= MOTOR_SELF_TEST_POS_TOL;
        bool vel_ok = abs((int32_t)derivative) <= MOTOR_SELF_TEST_VEL_TOL;

        // Abort if PID is commanding but encoder is not moving enough.
        if (abs(pos - prev_pos) >= MOTOR_SELF_TEST_STALL_POS_DELTA) {
            progress_ms = now_ms;
        } else if (now_ms - progress_ms > MOTOR_SELF_TEST_STALL_TIMEOUT_MS && abs(error) > MOTOR_SELF_TEST_POS_TOL) {
            set_motor_torque_direct(0, gain);
            Serial.println("Self-test segment aborted: stall detected.");
            return false;
        }

        if (pos_ok && vel_ok) {
            if (in_tolerance_since == 0) {
                in_tolerance_since = now_ms;
            }
            if (now_ms - in_tolerance_since >= settle_ms) {
                set_motor_torque_direct(0, gain);
                return true;
            }
        } else {
            in_tolerance_since = 0;
        }

        prev_error = error;
        prev_pos = pos;
        prev_ms = now_ms;
        delay(MOTOR_SELF_TEST_LOOP_MS);
    }

    set_motor_torque_direct(0, gain);
    return false;
}

void run_motor_self_test() {
    Serial.println("Motor self-test starting. Keep hands clear.");

    int16_t center = encoder_get_count();
    int16_t span = (int16_t)(ENCODER_MAX_COUNT * MOTOR_SELF_TEST_SPAN_RATIO);
    int16_t left_target = constrain(center - span, -ENCODER_MAX_COUNT, ENCODER_MAX_COUNT);
    int16_t right_target = constrain(center + span, -ENCODER_MAX_COUNT, ENCODER_MAX_COUNT);

    Serial.print("Self-test center: ");
    Serial.println(center);
    Serial.print("Self-test left target: ");
    Serial.println(left_target);
    Serial.print("Self-test right target: ");
    Serial.println(right_target);

    bool ok_left = run_motor_to_target_pid(
        left_target,
        MOTOR_SELF_TEST_GAIN,
        MOTOR_SELF_TEST_TIMEOUT_MS,
        MOTOR_SELF_TEST_SETTLE_MS
    );
    delay(200);

    bool ok_right = run_motor_to_target_pid(
        right_target,
        MOTOR_SELF_TEST_GAIN,
        MOTOR_SELF_TEST_TIMEOUT_MS,
        MOTOR_SELF_TEST_SETTLE_MS
    );
    delay(200);

    bool ok_center = run_motor_to_target_pid(
        center,
        MOTOR_SELF_TEST_GAIN,
        MOTOR_SELF_TEST_TIMEOUT_MS,
        MOTOR_SELF_TEST_SETTLE_MS
    );

    set_motor_torque_direct(0, MOTOR_SELF_TEST_GAIN);
    Serial.print("Self-test result L/R/C: ");
    Serial.print(ok_left ? "OK" : "TIMEOUT");
    Serial.print("/");
    Serial.print(ok_right ? "OK" : "TIMEOUT");
    Serial.print("/");
    Serial.println(ok_center ? "OK" : "TIMEOUT");
    Serial.println("Motor self-test finished.");
}

void on_hid_output_report(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    if (report_id == FFB_RID_VENDOR_TORQUE && len >= 3) {
        int16_t torque = (int16_t)(buffer[0] | (buffer[1] << 8));
        uint8_t gain = buffer[2];
        g_ffb_torque_cmd = constrain(torque, FFB_MIN_CMD, FFB_MAX_CMD);
        g_ffb_device_gain = gain;
        g_last_ffb_ms = millis();
    }
}

void encoder_init() {
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = ENCODER_PIN_A;
    pcnt_config.ctrl_gpio_num = ENCODER_PIN_B;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT_USED;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_UNIT_USED);
    pcnt_counter_clear(PCNT_UNIT_USED);
    pcnt_counter_resume(PCNT_UNIT_USED);
}

int16_t encoder_get_count() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_USED, &count);
    count = constrain(count, -ENCODER_MAX_COUNT, ENCODER_MAX_COUNT);
    return count;
}

void hall_init() {
    analogReadResolution(12);
    analogSetPinAttenuation(HALL_ACCEL_PIN, ADC_11db);
    analogSetPinAttenuation(HALL_BRAKE_PIN, ADC_11db);
    pinMode(HALL_ACCEL_PIN, INPUT);
    pinMode(HALL_BRAKE_PIN, INPUT);
}

void hall_calibrate(HallSensor &sensor, const char* name) {
    uint16_t value;
    Serial.print("Calibrating ");
    Serial.println(name);

    Serial.println("Set to MIN position in 3 seconds...");
    delay(3000);
    sensor.min_value = 0;
    for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
        value = analogRead(sensor.pin);
        if (value > sensor.min_value) {
            sensor.min_value = value;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }

    Serial.print("Min calibrated: ");
    Serial.println(sensor.min_value);

    Serial.println("Set to MAX position in 3 seconds...");
    delay(3000);
    sensor.max_value = 4095;
    for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
        value = analogRead(sensor.pin);
        if (value < sensor.max_value) {
            sensor.max_value = value;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }

    Serial.print("Max calibrated: ");
    Serial.println(sensor.max_value);
    Serial.println("Calibration complete.");
}

uint16_t hall_get_value(HallSensor &sensor) {
    uint16_t raw = analogRead(sensor.pin);
    raw = constrain(raw, sensor.min_value, sensor.max_value);
    return map(raw, sensor.min_value, sensor.max_value, 0, 65535);
}

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
        case MSG_TYPE_ENCODER:
        case MSG_TYPE_ANALOG:
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

void setup() {
    USB.begin();
    gamepad.setOutputCallback(on_hid_output_report);
    gamepad.begin();

    Serial.begin(115200);
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1, TXD1);

    motor_init();
    Serial.println("BTS7960B ready.");

    encoder_init();
    Serial.println("Encoder ready.");

    hall_init();
    Serial.println("Hall Calibration:");
    hall_calibrate(hall_accel, "Accelerator");
    hall_calibrate(hall_brake, "Brake");
    Serial.println("Hall sensors ready.");

#if RUN_MOTOR_SELF_TEST_AFTER_CALIB
    run_motor_self_test();
#endif

    g_last_ffb_ms = millis();
}

void loop() {
    handle_UART();

    set_motor_torque(g_ffb_torque_cmd);

    int16_t joyX = map(encoder_get_count(), ENCODER_MAX_COUNT, -ENCODER_MAX_COUNT, -32768, 32767);
    int16_t joyY = map(hall_get_value(hall_accel), 0, 65535, -32768, 32767);
    int16_t joyZ = map(hall_get_value(hall_brake), 0, 65535, -32768, 32767);

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

#if DEBUG_TELEMETRY
    uint32_t now = millis();
    if (now - g_last_debug_ms >= DEBUG_TELEMETRY_INTERVAL_MS) {
        g_last_debug_ms = now;
        Serial.print("Joystick X: ");
        Serial.print(joyX);
        Serial.print(" Y: ");
        Serial.print(joyY);
        Serial.print(" Z: ");
        Serial.print(joyZ);
        Serial.print(" FFB cmd: ");
        Serial.print(g_ffb_torque_cmd);
        Serial.print(" gain: ");
        Serial.println(g_ffb_device_gain);
    }
#endif

    delay(5);
}
