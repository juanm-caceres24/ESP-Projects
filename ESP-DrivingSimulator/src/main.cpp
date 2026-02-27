#include <Arduino.h>
#include "driver/pcnt.h"
#include "USB.h"
#include "USBHIDGamepad.h"

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

struct HallSensor {
    uint8_t pin;
    uint16_t min_value;
    uint16_t max_value;
};

USBHIDGamepad gamepad;
HallSensor hall_accel = {HALL_ACCEL_PIN, 0, 4095};
HallSensor hall_brake = {HALL_BRAKE_PIN, 0, 4095};
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
uint32_t hid_buttons = 0;

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

uint8_t hall_get_value(HallSensor &sensor) {
    uint16_t raw = analogRead(sensor.pin);
    raw = constrain(raw, sensor.min_value, sensor.max_value);
    return map(raw, sensor.min_value, sensor.max_value, 0, 127);
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
    gamepad.begin();
    Serial.begin(115200);
    Serial1.begin(UART_BAUD, SERIAL_8N1, RXD1, TXD1);
    encoder_init();
    Serial.println("Encoder ready.");
    hall_init();
    Serial.println("Hall Calibration:");
    hall_calibrate(hall_accel, "Accelerator");
    hall_calibrate(hall_brake, "Brake");
    Serial.println("Hall sensors ready.");
}

void loop() {
    handle_UART();
    int8_t joyX = map(encoder_get_count(), ENCODER_MAX_COUNT, -ENCODER_MAX_COUNT, -128, 127);
    int8_t joyY = map(hall_get_value(hall_accel), 0, 127, -128, 127);
    int8_t joyZ = map(hall_get_value(hall_brake), 0, 127, -128, 127);
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
    Serial.println(joyZ);
    delay(5);
}
