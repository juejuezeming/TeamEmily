#ifndef PADDLE_SERVO_H
#define PADDLE_SERVO_H

#include <Arduino.h>

// Call this in setup() before using paddle functions
void initPaddleServo(int servoPin);

void movePaddle(float degrees);
/**
 * Moves the paddle by a specific amount.
 * @param degrees the degrees of rotation (absolute) to move the paddle to
 */

void resetPaddle();
/**
 * Moves paddle to 0 point (home position).
 */

#endif