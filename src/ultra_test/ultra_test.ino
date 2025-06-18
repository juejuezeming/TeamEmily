#include <Arduino.h>

const int echoPin = 35;
const int trigPin = 34;
long duration;

int ultraSonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void setup() {
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
 Serial.begin(9600);
}
void loop() {
 int distance = ultraSonicDistance();
 Serial.println(distance);
 delay(1000);
}