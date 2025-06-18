#include "PaddleServo.h"
#include <Servo.h>

static Servo paddleServo;
static int servoPin = -1;
static float homePosition = 0; // degrees

void initPaddleServo(int pin) {
    servoPin = pin;
    paddleServo.attach(servoPin);
    resetPaddle();
}

void movePaddle(float degrees) {
    // Clamp degrees to [0, 180] for most servos
    if (degrees < 0) degrees = 0;
    if (degrees > 180) degrees = 180;
    paddleServo.write(degrees);
}

void resetPaddle() {
    movePaddle(homePosition);
}