#include <Arduino.h>

// Pins
#define ROTARY_ENCODER_A_PIN 12
#define ROTARY_ENCODER_B_PIN 13

// Rotary encoder parameters
#define ROTARY_ENCODER_INCREMENT 15 // Increment per detent (in grades 360 * 100)

// Global variables
uint8_t rotaryEncoderLastStates = 0b00; // Bit 0: A state, Bit 1: B state
uint32_t rotaryEncoderPosition = 0; // Position in grades 360 * 100
uint8_t updateEncoder = 0;

// Function prototypes
void rotaryEncoder_ISR();

void setup() {
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);
    Serial.println("ESP Driving Simulator Initialized");
    // Initialize GPIO pins for rotary encoder
    pinMode(ROTARY_ENCODER_A_PIN, INPUT); // A
    pinMode(ROTARY_ENCODER_B_PIN, INPUT); // B
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), rotaryEncoder_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), rotaryEncoder_ISR, CHANGE);
}

void loop() {
    if (updateEncoder) {
        Serial.print("Encoder Value: ");
        Serial.println(rotaryEncoderPosition);
        updateEncoder = 0;
    }
}

void rotaryEncoder_ISR() {
    // Combine states into a 2-bit value
    uint8_t currentStates = (digitalRead(ROTARY_ENCODER_A_PIN) << 1) | digitalRead(ROTARY_ENCODER_B_PIN);
    // Determine rotation direction
    if (rotaryEncoderLastStates == 0b00) {
        if (currentStates == 0b01) {
            rotaryEncoderPosition += ROTARY_ENCODER_INCREMENT; // Clockwise
            if (rotaryEncoderPosition >= 36000) {
                rotaryEncoderPosition = 0; // Wrap around
            }
        } else if (currentStates == 0b10) {
            if (rotaryEncoderPosition == 0) {
                rotaryEncoderPosition = 35985; // Wrap around
            } else {
                rotaryEncoderPosition -= ROTARY_ENCODER_INCREMENT; // Counter-clockwise
            }
        }
    } else if (rotaryEncoderLastStates == 0b01) {
        if (currentStates == 0b11) {
            rotaryEncoderPosition += ROTARY_ENCODER_INCREMENT; // Clockwise
            if (rotaryEncoderPosition >= 36000) {
                rotaryEncoderPosition = 0; // Wrap around
            }
        } else if (currentStates == 0b00) {
            if (rotaryEncoderPosition == 0) {
                rotaryEncoderPosition = 35985; // Wrap around
            } else {
                rotaryEncoderPosition -= ROTARY_ENCODER_INCREMENT; // Counter-clockwise
            }
        }
    } else if (rotaryEncoderLastStates == 0b11) {
        if (currentStates == 0b10) {
            rotaryEncoderPosition += ROTARY_ENCODER_INCREMENT; // Clockwise
            if (rotaryEncoderPosition >= 36000) {
                rotaryEncoderPosition = 0; // Wrap around
            }
        } else if (currentStates == 0b01) {
            if (rotaryEncoderPosition == 0) {
                rotaryEncoderPosition = 35985; // Wrap around
            } else {
                rotaryEncoderPosition -= ROTARY_ENCODER_INCREMENT; // Counter-clockwise
            }
        }
    } else if (rotaryEncoderLastStates == 0b10) {
        if (currentStates == 0b00) {
            rotaryEncoderPosition += ROTARY_ENCODER_INCREMENT; // Clockwise
            if (rotaryEncoderPosition >= 36000) {
                rotaryEncoderPosition = 0; // Wrap around
            }
        } else if (currentStates == 0b11) {
            if (rotaryEncoderPosition == 0) {
                rotaryEncoderPosition = 35985; // Wrap around
            } else {
                rotaryEncoderPosition -= ROTARY_ENCODER_INCREMENT; // Counter-clockwise
            }
        }
    }
    // Update last states and set update flag
    rotaryEncoderLastStates = currentStates;
    updateEncoder = 1;
}