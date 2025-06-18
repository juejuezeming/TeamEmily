

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

}


void shoot(){
  
}