#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Filters.h>
MPU6050 mpu;

/** Servo (ESC/Motor) variable **/
Servo esc;

/** Working variables **/
unsigned long lastTime;
double unfilteredPitch, pitch, desiredPitch, lastpitch, Mapped, Out;
double unfilteredRoll, roll;


double val = - 20;        //To set the default angle to -20
int SampleTime = 4;       //4 milli sec

float filterVal = 0.85;

/** filters out changes faster that number below (in Hz). **/
float filterFrequency = 4.0;

/** The PID gains and terms **/
float kp, ki, kd;
double PTerm, ITerm, DTerm, DTermUnfiltered;

int errorPitch;
char test[3];

void setup() {
  Serial.begin(9600);
  /** Bind ESC and motor to pin 9, then output value that doesn't make it run **/
  esc.attach(9);
  esc.write(1100);
  
  /** Testing connection to IMU **/
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
}

void loop() {
  /** Nothing will not run until 3 seconds in **/
  if (millis() < 3000) {
    Out = 1100;
  } else {
    /** Read normalized values from IMU **/
    Vector normAccel = mpu.readNormalizeAccel();
  
    /** Calculate pitch and roll from normalized values **/
    unfilteredPitch = (atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis)) * 180.0) / M_PI;
    //int unfilteredRoll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    
    pitch =  smooth(unfilteredPitch, filterVal, pitch);   // second parameter determines smoothness  - 0 is off,  .9999 is max smooth 

    /** Will read input angle from Serial Monitor **/
    int i = 0;
    while (Serial.available()) {
     test[i++] = Serial.read();
    }
    
    desiredPitch = atoi(test) + val;
    errorPitch  = desiredPitch - pitch;
  
    /** Reading potentiometer values and casting them in a 0-10 range **/
    kp = analogRead(A0);
    ki = analogRead(A1);
    kd = analogRead(A2);
    kp = map(kp, 0, 1023, 0, 3000) / 100000.0;
    ki = map(ki, 0, 1023, 0, 90) / 1000000.0;
    kd = map(kd, 0, 1023, 0, 100000) / 1000000.0;

    /** Reading the time since last reading **/
    unsigned long now = millis();
    int timeChange = (now - lastTime);

    if(timeChange >= SampleTime){
      /*Compute all the working error variables*/
      double errorPitch = desiredPitch - pitch;
      ITerm += (ki * errorPitch); 
      //Serial.print(" ITerm: ");
      //Serial.println(ITerm);
      
      double dpitch = (pitch - lastpitch);
      DTermUnfiltered = kd * dpitch;
      //Serial.print(" DTermUnfiltered: ");
      //Serial.print(DTermUnfiltered, 3);
      DTerm = smooth(DTermUnfiltered, 0.5, DTerm);
      //Serial.print(" DTerm: ");
      //Serial.println(DTerm, 3);
      
      /** Compute PID Mapped **/
      Mapped = kp * errorPitch + ITerm + DTerm; 
 
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
    Out = ((Mapped + 1) * (1400 - 1100)/ (1 + 1)) + 1000;
    //Out = map(Mapped, 1-, 1, 1100, 2000);
  }
  
  esc.write(Out);
  
  /** ---------- PRINTING SECTION (comment out what you don't need) ----------**/
  /** Unfiltered, filtered, desired pitch, and pitch error **/
  //Serial.print(" Unfiltered Pitch = ");
  //Serial.println(unfilteredPitch);
  //Serial.print(" Pitch = ");
  //Serial.print(pitch);
  //Serial.print(" Pitch_d = ");
  //Serial.println(desiredPitch);
  Serial.print(" Pitch_error = ");
  Serial.println(errorPitch);
  
  /** Gains and PID Terms **/
  //Serial.print("\t kp: ");
  //Serial.print(kp, 3);
  //Serial.print(" ki: ");
  //Serial.print(ki, 8);
  //Serial.print(" kd: ");
  //Serial.print(kd, 5);
  //Serial.print("\t P Term = ");
  //Serial.print(PTerm);
  //Serial.print(" I Term = ");
  //Serial.print(ITerm);
  //Serial.print(" D Term = ");
  //Serial.print(DTerm);

  /** Output after mapping...?? **/
  //Serial.print(" Mapped = ");
  //Serial.print(Mapped);
  //Serial.print(" Out = ");
  //Serial.println(Out);
}

int Mixer(int mapped) {
  int Out;
  Out = map(mapped, -1, 1, 1100, 2000);
  return Out;
}

double smooth(double data, float filterVal, float smoothedVal){
  /** Makes sure parameters are within range **/
  if (filterVal > 1) {
    filterVal = .99;
  } else if (filterVal <= 0){
    filterVal = 0;
  }

  /** Computes smoothed value **/
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal * filterVal);
  return (double) smoothedVal;
}

/** Initial attempt at filtering, kept here for reference, originally went right after unfiltered pitch reading **/
    // ------------------------------------
    // create a one pole (RC) lowpass filter
    //RuningStatistics inputStats;
    //intputStats.setWindowSecs( 10.0 );
    //FilterOnePole filterOneLowpass( LOWPASS, 1000000 );
    //RunningStatistics filterOneLowpassStats;
    //filterOneLowpassStats.setWindowSecs( 10.0);   
    //filterOneLowpass.input(unfilteredPitch);
    //pitch = filterOneLowpass.output();
    //pitch = unfilteredPitch;
    // ------------------------------------

