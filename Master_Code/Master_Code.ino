#include <Wire.h>
#include <MPU6050.h>
#include<Servo.h>

MPU6050 mpu;

/** Servo (ESC/Motor) variable **/
Servo esc;

/*working variables*/
unsigned long lastTime;
double pitch, ControlOut, desired;
double ITerm, lastpitch;
int SampleTime = 4; //4 milli sec
double outMin, outMax;

/* The PID gains, uncomment these three lines and comment out the fourth one if you want to manually adjust the gains */
//double kp = 1;
//double ki = 0;
//double kd = 0.1;
float kp, ki, kd;

int error;
char test[3];

void setup() {
  Serial.begin(9600);
  esc.attach(9);      //Servo is bound to pin 9
  esc.write(1000);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
}

void loop() {
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();
  
  // Calculate Pitch & Roll
  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  //int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  
  // Read desired angle angle from user
  int i = 0;
  if (Serial.available()) {
    //test[3] = {'0', '0', '0'}; 
  }
  while (Serial.available()) 
  {
   test[i++] = Serial.read();
  }
  
  desired = atoi(test);
  error  = desired - pitch;

  /* Reading potentiometer values and casting them in a 0-10 range */
  kp = analogRead(A0);
  ki = analogRead(A1);
  kd = analogRead(A2);
  kp = map(kp, 0, 1023, 0, 11) / 10.0;
  ki = map(ki, 0, 1023, 0, 11) / 10.0;
  kd = map(kd, 0, 1023, 0, 11) / 10.0;
  Serial.print("P-gain: ");
  Serial.print(kp, 2);
  Serial.print(", I-gain: ");
  Serial.print(ki, 2);
  Serial.print(", D-gain: ");
  Serial.print(kd, 2);
  
  unsigned long now = millis();
  int timeChange = (now - lastTime);
   
  if(timeChange >= SampleTime){
    /*Compute all the working error variables*/
    double error = desired - pitch;
    ITerm += (ki * error); 
    double dpitch = (pitch - lastpitch);
 
    /** Compute PID ControlOut **/
    ControlOut = kp * error + ITerm + kd * dpitch; 
 
    /** Remember some variables for next time **/
    lastpitch = pitch;
    lastTime = now;
  }
  // ControlOut
  Serial.print("\t");
  Serial.print(" Theta = ");
  Serial.print(pitch);
  Serial.print(" Theta_d = ");
  Serial.print(desired);
  Serial.print(" Error = ");
  Serial.print(error);
  Serial.print("\t ControlOut = ");
  Serial.println(ControlOut);
  delay(4);
  esc.write(desired*10);
}
