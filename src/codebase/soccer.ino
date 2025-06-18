#include <paddle.h>

const int maxWerte = 100;
uint16_t distances[maxWerte];
const treshhold;


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

void choose_where_to_shoot{
  int steps = 0;
  while(steps <= 7){
    if (sensor.readRangeSingleMillimeters() > treshhold){
        shoot();
        return;
    }
    steps++;
    rotate(steps, 1);
  }
  rotate(9, 1);
  shoot();
}



void shoot(){
  resetPaddle();
  if (sensor.readRangeSingleMillimeters() <= 20){
    movePaddle(180);
  }
}