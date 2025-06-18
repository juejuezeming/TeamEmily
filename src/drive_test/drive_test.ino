#include <Arduino.h>
#include <Encoder.h>

#include "pins_arduino.h"
#include <Servo.h>

#include <VL53L0X.h>


#define ENCODER_DO_NOT_USE_INTERRUPTS

Encoder myEnc(2,3);

VL53L0X sensor;


static int servoPin;
static Servo servo;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Wire.begin();
  Serial.println("Basic NoInterrupts Test:");

  servo.attach(servoPin);

  //set sensor pin

  sensor.init();
  sensor.setTimeout(500);

  sensor.setMeasurementTimingBudget(20000);

  

  
  Serial.println("Done Setup");

  //pinMode(ledPin, OUTPUT);

}



static const int ledPin = 11;


static long receivedNumber = 0;
void loop() {
  // put your main code here, to run repeatedly:

  int dist = sensor.readRangeSingleMillimeters();

  if(dist < 500){
  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.println(" mm");
  }

  delay(100);

  if (Serial.available()>0) {
    receivedNumber = Serial.parseInt();  // Reads an integer from serial
    Serial.print("Received: ");
    Serial.println(receivedNumber);

    if(receivedNumber > 0){
      analogWrite(ledPin,  receivedNumber);
      float fnum = (float) receivedNumber;
      //servo.write(fnum);

      //Serial.print("rotating servo to: ");
      //Serial.println(fnum);
      delay(100);

    }
  }

  

  

}
