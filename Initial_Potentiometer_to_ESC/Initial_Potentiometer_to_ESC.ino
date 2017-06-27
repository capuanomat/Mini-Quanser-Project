#include<Servo.h>

Servo esc;
//Servo esc2;
int throttlePin = 0;


void setup() {
  Serial.begin(9600);
  esc.attach(9);
  esc.write(1000);
  //esc2.attach(6);
}

int throttle = 1000;

void loop() {
  int incoming;
  int output;
  if (Serial.available() > 0) {
    incoming = Serial.read() - '0';
    output = 1000 + (100*incoming);
    Serial.println(output);
    esc.write(output);
    delay(1);
  }
  
  
  //int throttleNew = analogRead(throttlePin);
  //throttle = map(throttleNew, 0, 1023, 1000, 2000);
  //Serial.println(throttle);
  //esc.write(throttle);
  //esc2.write(throttle);
}





