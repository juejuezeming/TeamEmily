#include <Arduino.h>
#include <Encoder.h>

#include "pins_arduino.h"

#define ENCODER_DO_NOT_USE_INTERRUPTS

Encoder myEnc(2,3);

void setup() {
  Serial.begin(9600);
  Serial.println("Basic NoInterrupts Test:");
}

long position = -999;

void loop() {
  long newPos = myEnc.read();
  if (newPos != position) {
    position = newPos;
    Serial.println(position);
  }
  Serial.println(position);
  //delay(200);
  // With any substantial delay added, Encoder can only track
  // very slow motion.  You may uncomment this line to see
  // how badly a delay affects your encoder.
  //delay(50);
}
