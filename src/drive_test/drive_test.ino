#include <Arduino.h>
#include <Encoder.h>

#include "pins_arduino.h"

#define ENCODER_DO_NOT_USE_INTERRUPTS

Encoder myEnc(2,3);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println("Basic NoInterrupts Test:");

}

static const int ledPin = 2;

static long receivedNumber = 0;
void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()>0) {
    receivedNumber = Serial.parseInt();  // Reads an integer from serial
    Serial.print("Received: ");
    Serial.println(receivedNumber);

    if(receivedNumber > 0){
      analogWrite(ledPin, receivedNumber);

    }
  }

}
