#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;
Servo myservo;

int pos = 0;

void setup() {
  myservo.attach(9); 
  myservo.write(100);


  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  Serial.println("VL53L0X-Sensor bereit!");
}


void loop() {
  int distance = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.print("Timeout! ");
  }

  Serial.print("Abstand: ");
  Serial.print(distance);
  Serial.println(" mm");

  if (distance <= 130){
    Serial.println("shoot");
    myservo.write(20);
    for (pos = 0; pos <= 100; pos++) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(20);                       // waits 15 ms for the servo to reach the position
  }
  }

  delay(1000);
}