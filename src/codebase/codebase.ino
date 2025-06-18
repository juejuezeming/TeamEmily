#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "paddle.h"
#include "pins_arduino.h"

// Encoder Pin Definitions
static const int right_wheel_enc_1 = 2;
static const int right_wheel_enc_2 = 3;
static const int left_wheel_enc_1 = 4;
static const int left_wheel_enc_2 = 5;

// Encoder Instances
Encoder rightWheel(right_wheel_enc_1, right_wheel_enc_2);
Encoder leftWheel(left_wheel_enc_1, left_wheel_enc_2);

// VL53L0X Sensor and Servo
VL53L0X sensor;
Servo myservo;

// Pin-Definitionen for button/LED logic
const int buttonPins[2] = {A1, A2};
const int ledPins[3] = {6, 7, 8};  // Use 6,7,8 to avoid conflicts with encoder pins
const int executePin = 9;          // Use 9 to avoid conflicts with encoder pins

// State-Variable (1..3 = LEDs)
int state = 1;  // Start with State 1
// State 1 -> fußball
// State 2 -> golf
// State 3 -> slalom

void updateLEDs() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(ledPins[i], (state == i + 1) ? HIGH : LOW);
    }
}

void setup() {
    // Encoder pins are initialized by Encoder objects

    // LEDs as outputs
    for (int i = 0; i < 3; i++) {
        pinMode(ledPins[i], OUTPUT);
        digitalWrite(ledPins[i], LOW);
    }
    pinMode(executePin, OUTPUT);
    digitalWrite(executePin, LOW);

    // Buttons as input with pull-up
    for (int i = 0; i < 2; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }

    // Sensor and servo setup (if needed)
    Wire.begin();
    sensor.init();
    sensor.setTimeout(500);
    myservo.attach(10);  // Example pin, adjust as needed

    updateLEDs();  // Show initial state
}

void loop() {
    // Button 1 (A1): Durch die 3 States schalten
    if (digitalRead(buttonPins[0]) == LOW) {
        state++;
        if (state > 3) state = 1;  // States 1 bis 3 zyklisch
        updateLEDs();
        delay(200);  // Entprellzeit
    }

    // Button 2 (A2): Execute LED kurz einschalten
    if (digitalRead(buttonPins[1]) == LOW) {
        digitalWrite(executePin, HIGH);
        delay(2000);  // Execute LED 2 Sekunden an
        digitalWrite(executePin, LOW);
        delay(200);  // Entprellzeit
    }
}