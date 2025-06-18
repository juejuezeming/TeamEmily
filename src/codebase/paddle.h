#ifndef PADDLE_SERVO_H
#define PADDLE_SERVO_H

#include <Arduino.h>
#include <Servo.h>

// --- Internal static variables (to keep them only in this translation unit) ---
static Servo paddleServo;
static int servoPin = -1;
static float homePosition = 0; // degrees

// Call this in setup() before using paddle functions
void initPaddleServo(int pin) {
    servoPin = pin;
    paddleServo.attach(servoPin);
    resetPaddle();
}

/**
 * Moves the paddle by a specific amount.
 * @param degrees the degrees of rotation (absolute) to move the paddle to
 */
void movePaddle(float degrees) {
    // Clamp degrees to [0, 180] for most servos
    if (degrees < 0) degrees = 0;
    if (degrees > 180) degrees = 180;
    paddleServo.write(degrees);
}

/**
 * Moves paddle to 0 point (home position).
 */
void resetPaddle() {
    movePaddle(homePosition);
}

#endif // PADDLE_SERVO_H