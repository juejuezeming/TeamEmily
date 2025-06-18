#include <PID_v1.h>

#include <Encoder.h>
#include <Arduino.h>



#define right_forward_pin 8
#define right_backward_pin 9
#define left_forward_pin 11
#define left_backward_pin 10


#define speed 200
#define wheel_setoff 10

#define DEBUG
#define DEBUG3
//#define DEBUG_ENC
#define DEBUG2
#define DEBUGOUTPUT

#define ENCODER_USE_INTERRUPTS

Encoder right_enc(2,3);
Encoder left_enc(19,18);

double setpoint1, input1, output1;
double rp = 1, ri = 1, rd = 1; 
double setpoint2, input2, output2;
double lp = 1, li = 1, ld = 1;

PID pid_right(&input1, &output1, &setpoint1, rp, ri, rd, 0);
PID pid_left(&input2, &output2, &setpoint2, lp, li, ld, 0);

static const float ticks_per_cm = 1700; // TODO: measure this value
static const float steps_per_cm = 50.0; // TODO: measure this value
/***
 * Rotates the bot around a point on the axis of the wheels.
 * @param degrees the degrees of the rotation in degrees
 * @param distanceOnAxis the distance in cm from the center between the wheels, positive is to the right
 * @param speed the speed in %
 * @endcond rotation is done
 * @author Team Emily
 */
void rotate (float deg, float dist = 0, float s = 50){

  reset_l_PID();
  reset_r_PID();

  pid_right.SetMode(AUTOMATIC);
  pid_left.SetMode(AUTOMATIC);

  float left_wheel_circ = (wheel_setoff + dist) * 2 * PI;
  float right_wheel_circ = (-wheel_setoff + dist) * 2 * PI; 
  float middle_circ = dist * 2 * PI; // maybe for later


  float left_dist = left_wheel_circ * (deg / 360);
  float right_dist = right_wheel_circ * (deg / 360);

  #ifdef DEBUG
  Serial.print("Left Wheel Circumference: ");
  Serial.print(left_wheel_circ);
  Serial.print(" cm, Right Wheel Circumference: ");
  Serial.print(right_wheel_circ);
  Serial.print(" cm, Middle Circumference: ");
  Serial.print(middle_circ);
  Serial.println(" cm");
  Serial.print("Left Distance: ");
  Serial.print(left_dist);
  Serial.print(" cm, Right Distance: ");
  Serial.print(right_dist);
  Serial.println(" cm");
  #endif

  float total_dist = max(abs(left_dist), abs(right_dist));
  float left_ratio = left_dist / total_dist;
  float right_ratio = right_dist / total_dist;

  left_enc.write(0);
  right_enc.write(0);



  for(float step = 0; step <= total_dist; step+= .01){
    setpoint2 = (step * left_ratio);
    setpoint1 = (step * right_ratio);

    {
      #ifdef DEBUG3
      
      Serial.print("Setpoint Left: ");
      Serial.print(setpoint2);
      Serial.print(" cm, Setpoint Right: ");
      Serial.print(setpoint1);
      Serial.println(" cm");
      #endif
    }   

    input2 = ticks_to_cm(left_enc.read());
    input1 = ticks_to_cm(right_enc.read());

{
    #ifdef DEBUG2
    Serial.print("Input Left: ");
    Serial.print(input2);
    Serial.print(" cm, Input Right: ");
    Serial.print(input1);
    Serial.println(" cm");

    //Ticks
    Serial.print("Ticks Left: ");
    Serial.print(left_enc.read());
    Serial.print(", Ticks Right: ");
    Serial.print(right_enc.read());
    Serial.println();
    #endif
}
    pid_left.Compute();
    pid_right.Compute();

    #ifdef DEBUGOUTPUT
    Serial.print("Output Left: ");
    Serial.print(output2);
    Serial.print(", Output Right: ");
    Serial.println(output1);
    #endif

    
    if(output2 > 0 || true){
      analogWrite(right_forward_pin, constrain(abs(output2), 0, 255)); // TODO: some translation
      analogWrite(right_backward_pin, 0);
    }else{
      analogWrite(right_backward_pin, constrain(abs(output2), 0, 255));
      analogWrite(right_forward_pin, 0);
    }

  
    

    if(output1 > 0){
      analogWrite(left_forward_pin, constrain(abs(output1), 0, 255)); // TODO: some translation
      analogWrite(left_backward_pin, 0);
    }else{
      analogWrite(left_backward_pin, constrain(abs(output1), 0, 255));
      analogWrite(left_forward_pin, 0);  
    }

    delay(10);
  }
  
  analogWrite(left_forward_pin, 0);
  analogWrite(right_forward_pin, 255);
  analogWrite(left_backward_pin, 0);
  analogWrite(right_backward_pin, 255);

  #ifdef DEBUG
  Serial.println("Done Rotating");
  #endif

}

float ticks_to_cm(long count){
  return count / ticks_per_cm;
}
int cm_to_ticks(float cm){
  return cm * steps_per_cm;
}

void drive(float dist, float s = 50){

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
  Serial.begin(9600);

  Serial.println("Init done");

}

void loop() {
  // put your main code here, to run repeatedly:
  test_rotate_serial();

#ifdef DEBUG_ENC
  int re = right_enc.read();
  int le = left_enc.read();
  Serial.print("Right Encoder: ");
  Serial.print(re);
  Serial.print(", Left Encoder: ");
  Serial.println(le);
  #endif
  //delay(100);
}

void test_rotate_serial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    float deg = input.toFloat();
    if (deg != 0.0) {
      Serial.print("Rotating: ");
      Serial.println(deg);
      rotate(deg);
      Serial.println("Done rotating.");
    }
  }
  
}
