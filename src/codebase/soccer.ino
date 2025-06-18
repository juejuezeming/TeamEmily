#include <paddle.h>

const int maxWerte = 100;
uint16_t distances[maxWerte];


void drive_to_ball(){
    const abstand_zum_Ball;
    int distance = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) {
    Serial.print("Timeout! ");
    }
    abstand_zum_Ball = 20;
    drive(distance - 20);
}


void spot_goal_and_keeper(){
  rotate(90, -1);

  measurementIndex = 0;
  while (measurementIndex < maxWerte / 2){
    distances[measurementIndex] = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) {
      Serial.print("Timeout! ");
    }
    measurementIndex++;
  }
  rotate(90, 1);
  while(measurementIndex < maxWerte)
  {
    distances[measurementIndex] = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) {
      Serial.print("Timeout! ");
    }
    measurementIndex++;
  }

}


void shoot(){
  resetPaddle();
  if (sensor.readRangeSingleMillimeters() <= 20){
    movePaddle(180);
  }
}