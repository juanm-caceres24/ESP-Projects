#include <Arduino.h>

#define ADC_PIN 32
#define CALIB_SAMPLES 50
#define CALIB_DELAY_BETWEEN_SAMPLES 10

uint16_t value = 0;
uint16_t minValue = 0;
uint16_t maxValue = 4095;
uint16_t percentage = 0;

void calibrationWizard() {
    Serial.println("Starting calibration...");
    Serial.println("Please set the magnet in minimum position in 3 seconds...");
    delay(1000);
    for (uint8_t i = 0; i < 3; i++) {
        delay(1000);
        Serial.println(i + 1);
    }
    delay(1000);
    Serial.println("Calibrating minimum position...");
    for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
        value = analogRead(ADC_PIN);
        if (value > minValue) {
            minValue = value;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }
    Serial.println("Minimum position calibrated.");
    Serial.println("Please set the magnet in maximum position in 3 seconds...");
    delay(1000);
    for (uint8_t i = 0; i < 3; i++) {
        delay(1000);
        Serial.println(i + 1);
    }
    delay(1000);
    Serial.println("Calibrating maximum position...");
    for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
        value = analogRead(ADC_PIN);
        if (value < maxValue) {
            maxValue = value;
        }
        delay(CALIB_DELAY_BETWEEN_SAMPLES);
    }
    Serial.println("Maximum position calibrated.");
    Serial.println("Calibration complete.");
}

void setup() {
    // Serial port to display value of ADC pin
    Serial.begin(115200);
    Serial.println("Hello, ESP-HallSensorTest!");
    // ADC pin for Hall Sensor
    pinMode(ADC_PIN, INPUT);
    calibrationWizard();
}

void loop() {
    // Read Hall Sensor value from ADC pin
    value = analogRead(ADC_PIN);
    // Calculate percentage value
    if (value <= minValue) {
        percentage = 0; // If no valid range, set percentage to 0
    } else {
        percentage = map(value, minValue, maxValue, 0, 100);
        // Ensure percentage is within bounds
        if (percentage < 0) {
            percentage = 0;
        } else if (percentage > 100) {
            percentage = 100;
        }
    }
    // Print the value to Serial Monitor
    Serial.print("Hall Sensor Value percentage: ");
    Serial.println(percentage);
    // Wait before next reading
    delay(100);
}
