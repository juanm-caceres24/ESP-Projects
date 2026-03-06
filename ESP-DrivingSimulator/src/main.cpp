#include <Arduino.h>
#include <math.h>
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

// BTS7960B motor driver pins (adjust to your wiring).
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
#define MOTOR_SELF_TEST_TORQUE 320
#define MOTOR_SELF_TEST_GAIN 220
#define MOTOR_SELF_TEST_HOLD_MS 1200
#define MOTOR_SELF_TEST_SETTLE_MS 500
#define MOTOR_SELF_TEST_STEPS 24

#define FFB_TIMEOUT_MS 2000
#define FFB_MIN_CMD -1000
#define FFB_MAX_CMD 1000
#define FFB_DEADZONE 15

#ifndef USE_PID_DESCRIPTOR
#define USE_PID_DESCRIPTOR 0
#endif

#define FFB_MAX_EFFECTS 16
#if USE_PID_DESCRIPTOR
#define FFB_RID_SET_EFFECT 0x30
#define FFB_RID_SET_ENVELOPE 0x31
#define FFB_RID_SET_CONDITION 0x32
#define FFB_RID_SET_PERIODIC 0x33
#define FFB_RID_SET_CONSTANT_FORCE 0x34
#define FFB_RID_SET_RAMP_FORCE 0x35
#define FFB_RID_EFFECT_OPERATION 0x36
#define FFB_RID_BLOCK_FREE 0x37
#define FFB_RID_DEVICE_CONTROL 0x38
#define FFB_RID_DEVICE_GAIN 0x39
#define FFB_RID_CREATE_NEW_EFFECT_FEATURE 0x3A
#define FFB_RID_BLOCK_LOAD_FEATURE 0x3B
#define FFB_RID_PID_POOL_FEATURE 0x3C
#define FFB_RID_PID_STATE_INPUT 0x3D
#else
#define FFB_RID_SET_EFFECT 0x30
#define FFB_RID_SET_ENVELOPE 0x31
#define FFB_RID_SET_CONDITION 0x32
#define FFB_RID_SET_PERIODIC 0x33
#define FFB_RID_SET_CONSTANT_FORCE 0x34
#define FFB_RID_SET_RAMP_FORCE 0x35
#define FFB_RID_EFFECT_OPERATION 0x36
#define FFB_RID_BLOCK_FREE 0x37
#define FFB_RID_DEVICE_CONTROL 0x38
#define FFB_RID_DEVICE_GAIN 0x39
#define FFB_RID_CREATE_NEW_EFFECT_FEATURE 0x3A
#define FFB_RID_BLOCK_LOAD_FEATURE 0x3B
#define FFB_RID_PID_POOL_FEATURE 0x3C
#define FFB_RID_PID_STATE_INPUT 0x3D
#endif
#define FFB_RID_VENDOR_TORQUE 0x20

#define FFB_OP_START 0x01
#define FFB_OP_START_SOLO 0x02
#define FFB_OP_STOP 0x03

#define FFB_DC_ENABLE_ACTUATORS 0x01
#define FFB_DC_DISABLE_ACTUATORS 0x02
#define FFB_DC_STOP_ALL_EFFECTS 0x03
#define FFB_DC_RESET 0x04
#define FFB_DC_PAUSE 0x05
#define FFB_DC_CONTINUE 0x06

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
volatile bool g_vendor_ffb_override = false;
volatile int16_t g_vendor_ffb_torque_cmd = 0;
volatile uint8_t g_last_created_effect = 0;
int16_t g_last_encoder_count = 0;
uint32_t g_last_encoder_ms = 0;

struct FfbEffect {
    bool allocated;
    bool playing;
    uint8_t type;
    uint8_t gain;
    uint8_t direction;
    uint16_t duration;
    uint16_t start_delay;
    uint16_t sample_period;
    int16_t constant_magnitude;
    int16_t ramp_start;
    int16_t ramp_end;
    int16_t periodic_magnitude;
    int16_t periodic_offset;
    uint8_t periodic_phase;
    uint16_t periodic_period;
    uint16_t attack_time;
    uint16_t fade_time;
    uint16_t attack_level;
    uint16_t fade_level;
    int16_t cp_offset;
    int16_t pos_coeff;
    int16_t neg_coeff;
    uint16_t dead_band;
    uint16_t pos_sat;
    uint16_t neg_sat;
    uint32_t start_time_ms;
};

FfbEffect g_ffb_effects[FFB_MAX_EFFECTS] = {};
volatile uint8_t g_active_effect = 0;
volatile bool g_ffb_actuators_enabled = true;
volatile bool g_ffb_paused = false;

int16_t encoder_get_count();

static inline int32_t apply_saturation(int32_t value, uint16_t pos_sat, uint16_t neg_sat) {
    int32_t pos = (pos_sat == 0) ? FFB_MAX_CMD : map(pos_sat, 0, 65535, 0, FFB_MAX_CMD);
    int32_t neg = (neg_sat == 0) ? FFB_MAX_CMD : map(neg_sat, 0, 65535, 0, FFB_MAX_CMD);
    if (value >= 0) {
        return min(value, pos);
    }
    return max(value, -neg);
}

static inline int16_t ffb_coeff_to_cmd(int16_t coeff, int32_t input, int32_t divisor) {
    int32_t out = ((int32_t)coeff * input) / divisor;
    return (int16_t)constrain(out, FFB_MIN_CMD, FFB_MAX_CMD);
}

void stop_all_effects() {
    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
        g_ffb_effects[i].playing = false;
    }
    g_active_effect = 0;
    g_ffb_torque_cmd = 0;
}

int16_t ffb_effect_to_torque(uint8_t effect_index) {
    if (effect_index == 0 || effect_index > FFB_MAX_EFFECTS) {
        return 0;
    }

    FfbEffect &fx = g_ffb_effects[effect_index - 1];
    if (!fx.allocated || !fx.playing) {
        return 0;
    }

    int16_t encoder_now = encoder_get_count();
    uint32_t now = millis();
    int32_t dt_ms = (g_last_encoder_ms == 0) ? 5 : (int32_t)(now - g_last_encoder_ms);
    if (dt_ms <= 0) {
        dt_ms = 1;
    }
    int32_t velocity = ((int32_t)encoder_now - g_last_encoder_count) * 1000 / dt_ms;
    g_last_encoder_count = encoder_now;
    g_last_encoder_ms = now;

    int32_t elapsed = (int32_t)(now - fx.start_time_ms);
    if (elapsed < 0) {
        elapsed = 0;
    }

    if (fx.start_delay > 0 && elapsed < fx.start_delay) {
        return 0;
    }

    int32_t active_elapsed = elapsed - fx.start_delay;
    if (active_elapsed < 0) {
        active_elapsed = 0;
    }

    if (fx.duration != 0 && active_elapsed >= fx.duration) {
        fx.playing = false;
        return 0;
    }

    int32_t torque = 0;

    // Common effect types used by many DirectInput wheels.
    switch (fx.type) {
        case 1: // Constant
            torque = map(fx.constant_magnitude, -32768, 32767, FFB_MIN_CMD, FFB_MAX_CMD);
            break;
        case 2: // Ramp
            if (fx.duration == 0) {
                torque = map(fx.ramp_end, -32768, 32767, FFB_MIN_CMD, FFB_MAX_CMD);
            } else {
                int32_t start_cmd = map(fx.ramp_start, -32768, 32767, FFB_MIN_CMD, FFB_MAX_CMD);
                int32_t end_cmd = map(fx.ramp_end, -32768, 32767, FFB_MIN_CMD, FFB_MAX_CMD);
                int32_t t = constrain(active_elapsed, 0, (int32_t)fx.duration);
                torque = start_cmd + ((end_cmd - start_cmd) * t) / fx.duration;
            }
            break;
        case 3: // Square
        case 4: // Sine
        case 5: // Triangle
        case 6: // Sawtooth Up
        case 7: // Sawtooth Down
            {
                int32_t mag = map(fx.periodic_magnitude, -32768, 32767, FFB_MIN_CMD, FFB_MAX_CMD);
                int32_t off = map(fx.periodic_offset, -32768, 32767, FFB_MIN_CMD / 2, FFB_MAX_CMD / 2);
                uint16_t period = (fx.periodic_period == 0) ? 1000 : fx.periodic_period;
                float phase01 = ((active_elapsed % period) / (float)period) + (fx.periodic_phase / 255.0f);
                phase01 = phase01 - floorf(phase01);
                float wave = 0.0f;
                if (fx.type == 3) {
                    wave = (phase01 < 0.5f) ? 1.0f : -1.0f;
                } else if (fx.type == 4) {
                    wave = sinf(phase01 * 2.0f * PI);
                } else if (fx.type == 5) {
                    wave = 4.0f * fabsf(phase01 - 0.5f) - 1.0f;
                } else if (fx.type == 6) {
                    wave = 2.0f * phase01 - 1.0f;
                } else {
                    wave = 1.0f - 2.0f * phase01;
                }
                torque = off + (int32_t)(mag * wave);
            }
            break;
        case 8: // Spring
            {
                int32_t error = (int32_t)encoder_now - fx.cp_offset;
                if (abs(error) <= fx.dead_band) {
                    torque = 0;
                } else if (error > 0) {
                    torque = -ffb_coeff_to_cmd(fx.pos_coeff, error - fx.dead_band, 2048);
                } else {
                    torque = -ffb_coeff_to_cmd(fx.neg_coeff, error + fx.dead_band, 2048);
                }
                torque = apply_saturation(torque, fx.pos_sat, fx.neg_sat);
            }
            break;
        case 9: // Damper
            {
                if (abs(velocity) <= fx.dead_band) {
                    torque = 0;
                } else if (velocity > 0) {
                    torque = -ffb_coeff_to_cmd(fx.pos_coeff, velocity - fx.dead_band, 256);
                } else {
                    torque = -ffb_coeff_to_cmd(fx.neg_coeff, velocity + fx.dead_band, 256);
                }
                torque = apply_saturation(torque, fx.pos_sat, fx.neg_sat);
            }
            break;
        case 10: // Inertia
            {
                int32_t abs_v = abs(velocity);
                int32_t inertial = (abs_v * abs_v) / 256;
                if (velocity > 0) {
                    torque = -ffb_coeff_to_cmd(fx.pos_coeff, inertial, 1024);
                } else if (velocity < 0) {
                    torque = ffb_coeff_to_cmd(fx.neg_coeff, inertial, 1024);
                } else {
                    torque = 0;
                }
                torque = apply_saturation(torque, fx.pos_sat, fx.neg_sat);
            }
            break;
        case 11: // Friction
            {
                int32_t stick = max((int32_t)4, (int32_t)fx.dead_band);
                if (abs(velocity) > stick) {
                    torque = (velocity > 0) ? -((int32_t)fx.pos_coeff / 8) : ((int32_t)fx.neg_coeff / 8);
                } else {
                    torque = 0;
                }
                torque = apply_saturation(torque, fx.pos_sat, fx.neg_sat);
            }
            break;
        default:
            torque = map(fx.constant_magnitude, -32768, 32767, FFB_MIN_CMD, FFB_MAX_CMD);
            break;
    }

    torque = (torque * fx.gain) / 255;
    return (int16_t)constrain(torque, FFB_MIN_CMD, FFB_MAX_CMD);
}

void update_ffb_torque() {
    if (g_vendor_ffb_override) {
        g_ffb_torque_cmd = constrain(g_vendor_ffb_torque_cmd, FFB_MIN_CMD, FFB_MAX_CMD);
        return;
    }

    if (!g_ffb_actuators_enabled || g_ffb_paused) {
        g_ffb_torque_cmd = 0;
        return;
    }

    int32_t sum_torque = 0;
    for (uint8_t i = 1; i <= FFB_MAX_EFFECTS; i++) {
        sum_torque += ffb_effect_to_torque(i);
    }
    g_ffb_torque_cmd = (int16_t)constrain(sum_torque, FFB_MIN_CMD, FFB_MAX_CMD);
}

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

void ramp_motor_torque(int16_t from_torque, int16_t to_torque, uint16_t total_ms, uint8_t gain) {
    int32_t delta = (int32_t)to_torque - from_torque;
    uint16_t step_delay = max((uint16_t)1, (uint16_t)(total_ms / MOTOR_SELF_TEST_STEPS));

    for (uint8_t i = 0; i <= MOTOR_SELF_TEST_STEPS; i++) {
        int16_t current = from_torque + (int16_t)(delta * i / MOTOR_SELF_TEST_STEPS);
        set_motor_torque_direct(current, gain);
        delay(step_delay);
    }
}

void run_motor_self_test() {
    Serial.println("Motor self-test starting. Keep hands clear.");

    ramp_motor_torque(0, MOTOR_SELF_TEST_TORQUE, 700, MOTOR_SELF_TEST_GAIN);
    delay(MOTOR_SELF_TEST_HOLD_MS);
    ramp_motor_torque(MOTOR_SELF_TEST_TORQUE, 0, 500, MOTOR_SELF_TEST_GAIN);
    delay(MOTOR_SELF_TEST_SETTLE_MS);

    ramp_motor_torque(0, -MOTOR_SELF_TEST_TORQUE, 700, MOTOR_SELF_TEST_GAIN);
    delay(MOTOR_SELF_TEST_HOLD_MS);
    ramp_motor_torque(-MOTOR_SELF_TEST_TORQUE, 0, 500, MOTOR_SELF_TEST_GAIN);

    set_motor_torque_direct(0, MOTOR_SELF_TEST_GAIN);
    Serial.println("Motor self-test finished.");
}

void on_hid_output_report(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    switch (report_id) {
        case FFB_RID_SET_EFFECT:
            g_vendor_ffb_override = false;
            if (len >= 7) {
                uint8_t idx = buffer[0];
                uint8_t type = buffer[1];
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    FfbEffect &fx = g_ffb_effects[idx - 1];
                    fx.allocated = true;
                    fx.type = type;
                    fx.duration = (uint16_t)(buffer[2] | (buffer[3] << 8));
                    fx.start_delay = (uint16_t)(buffer[4] | (buffer[5] << 8));
                    fx.gain = buffer[6];
                    if (fx.gain == 0) {
                        fx.gain = 255;
                    }
                }
            }
            break;
        case FFB_RID_SET_ENVELOPE:
            g_vendor_ffb_override = false;
            if (len >= 9) {
                uint8_t idx = buffer[0];
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    FfbEffect &fx = g_ffb_effects[idx - 1];
                    fx.attack_level = (uint16_t)(buffer[1] | (buffer[2] << 8));
                    fx.fade_level = (uint16_t)(buffer[3] | (buffer[4] << 8));
                    fx.attack_time = (uint16_t)(buffer[5] | (buffer[6] << 8));
                    fx.fade_time = (uint16_t)(buffer[7] | (buffer[8] << 8));
                }
            }
            break;
        case FFB_RID_SET_CONDITION:
            g_vendor_ffb_override = false;
            if (len >= 13) {
                uint8_t idx = buffer[0];
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    FfbEffect &fx = g_ffb_effects[idx - 1];
                    fx.cp_offset = (int16_t)(buffer[1] | (buffer[2] << 8));
                    fx.pos_coeff = (int16_t)(buffer[3] | (buffer[4] << 8));
                    fx.neg_coeff = (int16_t)(buffer[5] | (buffer[6] << 8));
                    fx.pos_sat = (uint16_t)(buffer[7] | (buffer[8] << 8));
                    fx.neg_sat = (uint16_t)(buffer[9] | (buffer[10] << 8));
                    fx.dead_band = (uint16_t)(buffer[11] | (buffer[12] << 8));
                }
            }
            break;
        case FFB_RID_SET_PERIODIC:
            g_vendor_ffb_override = false;
            if (len >= 9) {
                uint8_t idx = buffer[0];
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    FfbEffect &fx = g_ffb_effects[idx - 1];
                    fx.periodic_magnitude = (int16_t)(buffer[1] | (buffer[2] << 8));
                    fx.periodic_offset = (int16_t)(buffer[3] | (buffer[4] << 8));
                    fx.periodic_phase = buffer[5];
                    fx.periodic_period = (uint16_t)(buffer[6] | (buffer[7] << 8));
                    fx.sample_period = buffer[8];
                }
            }
            break;
        case FFB_RID_SET_CONSTANT_FORCE:
            g_vendor_ffb_override = false;
            if (len >= 3) {
                uint8_t idx = buffer[0];
                int16_t magnitude = (int16_t)(buffer[1] | (buffer[2] << 8));
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    FfbEffect &fx = g_ffb_effects[idx - 1];
                    fx.allocated = true;
                    fx.constant_magnitude = magnitude;
                    if (fx.gain == 0) {
                        fx.gain = 255;
                    }
                }
            }
            break;
        case FFB_RID_SET_RAMP_FORCE:
            g_vendor_ffb_override = false;
            if (len >= 5) {
                uint8_t idx = buffer[0];
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    FfbEffect &fx = g_ffb_effects[idx - 1];
                    fx.ramp_start = (int16_t)(buffer[1] | (buffer[2] << 8));
                    fx.ramp_end = (int16_t)(buffer[3] | (buffer[4] << 8));
                }
            }
            break;
        case FFB_RID_EFFECT_OPERATION:
            g_vendor_ffb_override = false;
            if (len >= 2) {
                uint8_t idx = buffer[0];
                uint8_t op = buffer[1];
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    if (op == FFB_OP_START || op == FFB_OP_START_SOLO) {
                        if (op == FFB_OP_START_SOLO) {
                            stop_all_effects();
                        }
                        g_ffb_effects[idx - 1].playing = true;
                        g_ffb_effects[idx - 1].start_time_ms = millis();
                        g_active_effect = idx;
                    } else if (op == FFB_OP_STOP) {
                        g_ffb_effects[idx - 1].playing = false;
                        if (g_active_effect == idx) {
                            g_active_effect = 0;
                        }
                    }
                }
            }
            break;
        case FFB_RID_BLOCK_FREE:
            g_vendor_ffb_override = false;
            if (len >= 1) {
                uint8_t idx = buffer[0];
                if (idx >= 1 && idx <= FFB_MAX_EFFECTS) {
                    g_ffb_effects[idx - 1] = {};
                    if (g_active_effect == idx) {
                        g_active_effect = 0;
                    }
                }
            }
            break;
        case FFB_RID_DEVICE_CONTROL:
            g_vendor_ffb_override = false;
            if (len >= 1) {
                uint8_t control = buffer[0];
                if (control == FFB_DC_ENABLE_ACTUATORS) {
                    g_ffb_actuators_enabled = true;
                } else if (control == FFB_DC_DISABLE_ACTUATORS) {
                    g_ffb_actuators_enabled = false;
                    stop_all_effects();
                } else if (control == FFB_DC_STOP_ALL_EFFECTS) {
                    stop_all_effects();
                } else if (control == FFB_DC_RESET) {
                    stop_all_effects();
                    for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
                        g_ffb_effects[i] = {};
                    }
                    g_ffb_actuators_enabled = true;
                    g_ffb_paused = false;
                } else if (control == FFB_DC_PAUSE) {
                    g_ffb_paused = true;
                } else if (control == FFB_DC_CONTINUE) {
                    g_ffb_paused = false;
                }
            }
            break;
        case FFB_RID_DEVICE_GAIN:
            if (len >= 2) {
                uint16_t gain16 = (uint16_t)(buffer[0] | (buffer[1] << 8));
                gain16 = constrain(gain16, 0, 255);
                g_ffb_device_gain = (uint8_t)gain16;
            }
            break;
        case FFB_RID_VENDOR_TORQUE:
            if (len >= 3) {
                int16_t torque = (int16_t)(buffer[0] | (buffer[1] << 8));
                uint8_t gain = buffer[2];
                g_vendor_ffb_override = true;
                g_vendor_ffb_torque_cmd = constrain(torque, FFB_MIN_CMD, FFB_MAX_CMD);
                g_ffb_device_gain = gain;
            }
            break;
        default:
            break;
    }

    update_ffb_torque();
    g_last_ffb_ms = millis();
}

uint16_t on_hid_get_feature(uint8_t report_id, uint8_t* buffer, uint16_t len) {
    if (report_id == FFB_RID_BLOCK_LOAD_FEATURE && len >= 4) {
        buffer[0] = g_last_created_effect;
        buffer[1] = 0x01; // load success
        uint16_t avail = 0xFFFF;
        buffer[2] = (uint8_t)(avail & 0xFF);
        buffer[3] = (uint8_t)((avail >> 8) & 0xFF);
        return 4;
    }
    if (report_id == FFB_RID_PID_POOL_FEATURE && len >= 4) {
        uint16_t ram_pool = 0xFFFF;
        buffer[0] = (uint8_t)(ram_pool & 0xFF);
        buffer[1] = (uint8_t)((ram_pool >> 8) & 0xFF);
        buffer[2] = FFB_MAX_EFFECTS;
        buffer[3] = 0x03;
        return 4;
    }
    return 0;
}

void on_hid_set_feature(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    if (report_id == FFB_RID_CREATE_NEW_EFFECT_FEATURE && len >= 2) {
        uint8_t requested_type = buffer[0];
        (void)requested_type;
        for (uint8_t i = 0; i < FFB_MAX_EFFECTS; i++) {
            if (!g_ffb_effects[i].allocated) {
                g_ffb_effects[i] = {};
                g_ffb_effects[i].allocated = true;
                g_ffb_effects[i].gain = 255;
                g_last_created_effect = i + 1;
                return;
            }
        }
        g_last_created_effect = 0;
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

void encoder_reset() {
    pcnt_counter_clear(PCNT_UNIT_USED);
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
                if (value)
                    hid_buttons |= (1 << BUTTON_A);
                else
                    hid_buttons &= ~(1 << BUTTON_A);
            }
            if (id == 1) {
                if (value)
                    hid_buttons |= (1 << BUTTON_B);
                else
                    hid_buttons &= ~(1 << BUTTON_B);
            }
            break;
        case MSG_TYPE_ENCODER:
            Serial.print("Encoder ");
            Serial.print(id);
            Serial.print(" -> ");
            Serial.println(value);
            break;
        case MSG_TYPE_ANALOG:
            Serial.print("Analog ");
            Serial.print(id);
            Serial.print(" -> ");
            Serial.println(value);
            break;
        default:
            Serial.println("Unknown packet type");
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
        }
        else {
            rxBuffer[rxIndex++] = byteReceived;
            if (rxIndex >= PACKET_SIZE) {
                uint8_t receivedCRC = rxBuffer[5];
                uint8_t calculatedCRC = calculate_CRC(rxBuffer);
                if (receivedCRC == calculatedCRC) {
                    process_packet(rxBuffer);
                } else {
                    Serial.println("CRC error");
                }
                rxIndex = 0;
            }
        }
    }
}

void setup() {
    USB.begin();
    gamepad.setOutputCallback(on_hid_output_report);
    gamepad.setGetFeatureCallback(on_hid_get_feature);
    gamepad.setSetFeatureCallback(on_hid_set_feature);
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
}

void loop() {
    handle_UART();
    update_ffb_torque();
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
    Serial.print("Joystick X: (");
    Serial.print(encoder_get_count());
    Serial.print(") ");
    Serial.print(joyX);
    Serial.print(" Y: ");
    Serial.print(joyY);
    Serial.print(" Z: ");
    Serial.print(joyZ);
    Serial.print(" FFB cmd: ");
    Serial.print(g_ffb_torque_cmd);
    Serial.print(" devGain: ");
    Serial.print(g_ffb_device_gain);
    Serial.print(" activeFx: ");
    Serial.println(g_active_effect);
    delay(5);
}
