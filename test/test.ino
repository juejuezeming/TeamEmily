#include "Arduino.h"



#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define D8 8
#define D9 9
#define D10 10
#define D11 11
#define D12 12


void setup() {
  // put your setup code here, to run once:
  pinMode(D12, INPUT);
  pinMode(D11, INPUT);
  pinMode(D10, INPUT);
  pinMode(D9, INPUT);

  pinMode(D7, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D6, OUTPUT);

  digitalWrite(D7, LOW);
  digitalWrite(D4, LOW);
  digitalWrite(D3, LOW);
  digitalWrite(D6, LOW);

  delay(1000);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(D7, HIGH);
  //digitalWrite(D4, HIGH);
  digitalWrite(D3, HIGH);
  //digitalWrite(D6, HIGH);

  delay(10000);

  digitalWrite(D7, LOW);
  digitalWrite(D4, LOW);
  digitalWrite(D3, LOW);
  digitalWrite(D6, LOW);

  delay(1000);
}
