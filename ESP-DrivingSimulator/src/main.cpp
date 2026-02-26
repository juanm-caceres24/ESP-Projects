#include <Arduino.h>
#include "driver/pcnt.h"

#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define PCNT_UNIT_USED PCNT_UNIT_0

#define HALL_ACCEL_PIN 14
#define HALL_BRAKE_PIN 13
#define CALIB_SAMPLES 50
#define CALIB_DELAY_BETWEEN_SAMPLES 10

struct HallSensor {
    uint8_t pin;
    uint16_t min_value;
    uint16_t max_value;
};

HallSensor hall_accel = {HALL_ACCEL_PIN, 0, 4095};
HallSensor hall_brake = {HALL_BRAKE_PIN, 0, 4095};

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
    Serial.println("=================================");
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

uint8_t hall_get_percentage(HallSensor &sensor) {
    uint16_t raw = analogRead(sensor.pin);
    if (raw <= sensor.min_value)
        return 0;
    if (raw >= sensor.max_value)
        return 100;
    return map(raw, sensor.min_value, sensor.max_value, 0, 100);
}

void setup() {
    Serial.begin(115200);
    encoder_init();
    Serial.println("Encoder ready.");
    hall_init();
    Serial.println("Hall Calibration:");
    hall_calibrate(hall_accel, "Accelerator");
    hall_calibrate(hall_brake, "Brake");
    Serial.println("Hall sensors ready.");
}

void loop() {
    int16_t position = encoder_get_count();
    uint8_t accel = hall_get_percentage(hall_accel);
    uint8_t brake = hall_get_percentage(hall_brake);
    Serial.println("Encoder position: " + String(position) + " | Accelerator: " + String(accel) + "% | Brake: " + String(brake) + "%");
    delay(100);
}
