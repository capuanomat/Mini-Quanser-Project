#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Filters.h>
MPU6050 mpu;

/** Servo (ESC/Motor) variable **/
Servo esc;

/*working variables*/
unsigned long lastTime;
double pitch, Mapped, Out, desired;
double pitchUnfiltered;
double val = - 20;
double PTerm, ITerm, DTerm, lastpitch;
int SampleTime = 4; //4 milli sec
double outMin, outMax;

// filters out changes faster that 5 Hz.
float filterFrequency = 5.0;

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
  esc.write(1100);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
}

void loop() {
  if (millis() < 3000) {
    /** Will not run until 5 seconds **/
    Out = 1100;
  } else {
    // Read normalized values 
    Vector normAccel = mpu.readNormalizeAccel();
  
    // Calculate Pitch & Roll
    pitchUnfiltered = (atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis)) * 180.0) / M_PI;
    //int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    
    // -----------------------------------    
    // create a one pole (RC) lowpass filter
    FilterOnePole lowpassFilter( LOWPASS, filterFrequency );   
    
    while( true ) {
      lowpassFilter.input(pitchUnfiltered);
      pitch = lowpassFilter.output();
      // do something else
    }
    // -----------------------------------
    

    // Read desired angle angle from user
    int i = 0;
    if (Serial.available()) {
      //test[3] = {'0', '0', '0'}; 
    }
    while (Serial.available()) 
    {
     test[i++] = Serial.read();
    }
    
    desired = atoi(test) + val;
    error  = desired - pitch;
  
    /** Reading potentiometer values and casting them in a 0-10 range **/
    kp = analogRead(A0);
    ki = analogRead(A1);
    kd = analogRead(A2);
    kp = map(kp, 0, 1023, 0, 5000) / 100000.0;
    ki = map(ki, 0, 1023, 0, 900) / 100000.0;
    kd = map(kd, 0, 1023, 0, 900) / 100000.0;
    //Serial.print("P: ");
    //Serial.print(kp, 3);
    //Serial.print(", I: ");
    //Serial.print(ki, 3);
    //Serial.print(", D: ");
    //Serial.print(kd, 3);

    /** Reading the time since last reading **/
    unsigned long now = millis();
    int timeChange = (now - lastTime);

    if(timeChange >= SampleTime){
      /*Compute all the working error variables*/
      double error = desired - pitch;
      ITerm += (ki * error); 
      
      double dpitch = (pitch - lastpitch);
      DTerm = kd * dpitch;
      
      /** Compute PID Mapped **/
      Mapped = kp * error + ITerm + DTerm; 
 
      /** Remember some variables for next time **/
      lastpitch = pitch;
      lastTime = now;
    }

    /** Error from -87 to 43 is mapped from -1 to 1**/
    //Mapped = map(Mapped, -87, 44, 1000, 1370);
    //Mapped = map(Mapped, -87, 43, -1, 1);
    if (Mapped > 1) {
      Mapped = 1;
    } else if (Mapped < -1) {
      Mapped = -1;
    }

    /** Input from -1 to 1 mapped to 1100 to 2000 **/
    Out = ((Mapped + 1) * (1500 - 1100)/ (1 + 1)) + 1000;
    //Out = map(Mapped, 1-, 1, 1100, 2000);
  }
  esc.write(Out);
  
  /** Prints the following: current pitch, desired pitch, error in pitch, PID output before mapping **/  
  Serial.print(" Pitch = ");
  Serial.println(pitch);
  //Serial.print(" Pitch_d = ");
  //Serial.println(desired);
  //Serial.print(" Error = ");
  //Serial.println(error);
  //Serial.print("\t I Term = ");
  //Serial.print(ITerm);
  //Serial.print(" D Term = ");
  //Serial.print(DTerm);
  //Serial.print(" Mapped = ");
  //Serial.print(Mapped);
  //Serial.print(" Out = ");
  //Serial.println(Out);
  delay(4); 
}

int Mixer(int mapped) {
  int Out;
  Out = map(mapped, -1, 1, 1100, 2000);
  return Out;
}

