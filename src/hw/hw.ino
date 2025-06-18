#include <PID_v1.h>

#include <Encoder.h>
#include <Arduino.h>



#define right_forward_pin 2
#define right_backward_pin 3
#define left_forward_pin 4
#define left_backward_pin 5


#define speed 200
#define wheel_setoff


Encoder right_enc(18,19);
Encoder left_enc(20,21);

double setpoint1, input1, output1;
double rp = 1, ri = 1, rd = 1; 
double setpoint2, input2, output2;
double lp = 1, li = 1, ld = 1;

PID pid_right(&setpoint1, &input1, &output1, rp, ri, rd, 2);
PID pid_left(&setpoint2, &input2, &output2, lp, li, ld, 2);

static const float ticks_per_cm;

/***
 * Rotates the bot around a point on the axis of the wheels.
 * @param degrees the degrees of the rotation in degrees
 * @param distanceOnAxis the distance in cm from the center between the wheels, positive is to the right
 * @param speed the speed in %
 * @endcond rotation is done
 * @author Team Emily
 */
void rotate (float deg, float dist, float s){

  reset_l_PID();
  reset_r_PID();

  pid_right.SetMode(AUTOMATIC);
  pid_left.SetMode(AUTOMATIC);

  float left_wheel_circ = (wheel_setoff + dist) * 2 * PI;
  float right_wheel_circ = (wheel_setoff - dist) * 2 * PI; 
  float middle_circ = dist * 2 * PI; // maybe for later


  float left_dist = left_wheel_circ * (deg / 360);
  float right_dist = right_wheel_circ * (deg / 360);

  float total_dist = max(abs(left_dist), abs(right_dist));
  float left_ratio = left_dist / total_dist;
  float right_ratio = right_dist / total_dist;

  long left_start = left_enc.read();
  long right_start = right_enc.read();

  for(long step = 0; step <= total_dist * ticks_per_cm; step++){
    setpoint2 = left_start + step * left_ratio;
    setpoint1 = right_start + step * right_ratio;

    input2 = left_enc.read();
    input1 = right_enc.read();

    pid_left.Compute();
    pid_right.Compute();

    if(output2 > 0)
      analogWrite(left_forward_pin, constrain(abs(output2), 0, 255)); // TODO: some translation
    else
      analogWrite(left_backward_pin, constrain(abs(output2), 0, 255));
    
    if(output1 > 0)
      analogWrite(right_forward_pin, constrain(abs(output1), 0, 255)); // TODO: some translation
    else
      analogWrite(right_backward_pin, constrain(abs(output1), 0, 255));

    delay(10);
  }
  
  analogWrite(left_forward_pin, 0);
  analogWrite(right_forward_pin, 0);
  

}

void reset_r_PID(){
  input1 = 0;
  output1 = 0;
  setpoint1 = 0;
}

void reset_l_PID(){
  input2 = 0;
  output2 = 0;
  setpoint2 = 0;
}


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
