#include <Arduino.h>
#include <Encoder.h>

#include "pins_arduino.h"

static const int right_wheel_enc_1 = 2;
static const int right_wheel_enc_2 = 3;

static const int left_wheel_enc_1 = 4;
static const int left_wheel_enc_2 = 5;




Encoder rightWheel(right_wheel_enc_1, right_wheel_enc_2);
Encoder leftWheel(left_wheel_enc_1, left_wheel_enc_2);


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  

}
