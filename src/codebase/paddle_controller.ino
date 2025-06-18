#include "PaddleServo.h"

const int paddleServoPin = 9; // Example pin

void setup() {
    initPaddleServo(paddleServoPin);
}

void loop() {
    movePaddle(60);   // Move paddle to 60°
    delay(1000);
    resetPaddle();    // Move back to 0°
    delay(1000);
}