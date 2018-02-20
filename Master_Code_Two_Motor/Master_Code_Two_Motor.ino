#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Filters.h>


/** Servo (ESC/Motor) variable and IMU variable **/
Servo escLeft;
Servo escRight;
MPU6050 mpu;

/** Working variables **/
unsigned long lastTime;
double unfilteredPitch, pitch, desiredPitch, errorPitch, lastPitch;
double unfilteredRoll, roll;

/** ADJUSTABLE: Setting a default pitch angle, sampling time in milliseconds, filter value, and  **/
double defaultAngle = - 10;
int SampleTime = 4;
float filterVal = 0.99;

/** The PID gains and PID terms **/
double kp, ki, kd;
double PTerm, ITerm, DTerm, DTermUnfiltered;
double ITermMin = 0;
double ITermMax = 2;
double sumPID;

/** To read serial monitor input angles **/
char readMonitor[3];  //CHECKON: changed this from "test" which was a keyword

/** Output, as well as it's minimum and maximum possible values to avoid integral wind up/overshoot **/
double Out;

void setup() {
  Serial.begin(9600);
  /** Bind ESC and motor to pin 9, then output value that doesn't make it run **/
  escLeft.attach(9);
  escLeft.write(1100);
  escRight.attach(10);
  escRight.write(1100);
  
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
    /** Read normalized values from IMU and caclulates pitch and roll from them**/
    Vector normAccel = mpu.readNormalizeAccel();
    unfilteredPitch = (atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis)) * 180.0) / M_PI;
    //int unfilteredRoll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

    /** Smoothens out the pitch (tried lowpass filter). Second parameter determines the smoothness: 0 is off, 0.9999 is max smoothness **/
    pitch =  smooth(unfilteredPitch, filterVal, pitch);
    //roll = smooth(unfilteredRoll, filterVal, roll); //May need separate filter values for pitch and roll

    /** Will read input angle from Serial Monitor **/
    int i = 0;
    while (Serial.available()) {
     readMonitor[i++] = Serial.read();
    }

    /** Compute desired pitch and adds val (-20) to give val default angle, then computes error in pitch **/
    desiredPitch = atoi(readMonitor) + defaultAngle;
    errorPitch  = desiredPitch - pitch;

    /** TODO: Does same as above for roll but without default angle **/
    // This will involve editing how values are read in so you can input both roll and pitch
  
    /** Reading the PID gains from potentiometers on Analog ports 0, 1, and 2. Casting them to reasonable ranges **/
    double pot1 = analogRead(A0);
    double pot2 = analogRead(A1);
    double pot3 = analogRead(A2);
    double potMin = 0;
    double potMax = 1023;
    double pMin = 0;
    double iMin = 0;
    double dMin = 0;
    double pMax = 0.01;
    double iMax = 0.0002;
    double dMax = 0.005;    
    kp = (((pot1 - potMin) * (pMax - pMin)) / (potMax - potMin)) + pMin;
    ki = (((pot2 - potMin) * (iMax - iMin)) / (potMax - potMin)) + iMin;
    kd = (((pot3 - potMin) * (dMax - dMin)) / (potMax - potMin)) + dMin;
        
    /** Calculating the time since last reading **/
    unsigned long now = millis();
    int timeChange = (now - lastTime);

    /** Computing the PID terms if timeChange is large enough **/
    if(timeChange >= SampleTime){
      /** P **/
      PTerm  = kp * errorPitch; // Proportional to instantaneous error in pitch
      
      /** I and clamping to prevent Integral wind up **/
      ITerm += ki * errorPitch; // Summed error in pitch over time
      if (ITerm > ITermMax) {
        ITerm = ITermMax;
      } else if (ITerm < ITermMin) {
        ITerm = ITermMin;
      }

      /** D **/
      double dpitch = (pitch - lastPitch);
      DTermUnfiltered = kd * dpitch;
      DTerm = smooth(DTermUnfiltered, 0.5, DTerm); // Instantaneous rate of change of pitch
      
      /** Compute sum of P, I, and D terms **/
      sumPID = PTerm + ITerm + DTerm; 

      /** Remember some variables for next time **/
      lastPitch = pitch;
      lastTime = now;
    }

    /** Error from -87 to 43 is mapped from -1 to 1**/
    //Mapped = map(Mapped, -87, 44, 1000, 1370);  // NOTE: Mapped was changed to sumPID
    //Mapped = map(Mapped, -87, 43, -1, 1);
    double controlOutMin = 0;
    double controlOutMax = 1;
    double mixMin = 1150;
    double mixMax = 2000;
    if (sumPID > controlOutMax) {
      sumPID = controlOutMax;
    } else if (sumPID < controlOutMin) {
      sumPID = controlOutMin;
    }

    /** Input from -1 to 1 mapped to 1100 to 2000 **/
    Out = (((sumPID - controlOutMin) * (mixMax - mixMin)) / (controlOutMax - controlOutMin)) + mixMin;
  }

  // Currently keeping both outputs identical for preliminary fixed roll testing with two motors
  escLeft.write(Out);
  escRight.write(Out);
  printGains();
  //printPID();
  //printPitch();
  //printError();
  //printOut();
}

//double linInterp(double input, 

double smooth(double data, float filterVal, double lastData){
  /** Makes sure parameters are within range **/
  if (filterVal > 1) {
    filterVal = .99;
  } else if (filterVal <= 0){
    filterVal = 0;
  }

  /** Computes smoothed value by taking into acount**/
  //TODO: Read up on where this equation came from (makes sense though)
  double smoothedVal = (data * (1 - filterVal)) + (lastData * filterVal);
  return smoothedVal;
}


void printPitch() {
  Serial.print(" Unfiltered Pitch = ");
  Serial.println(unfilteredPitch);
  Serial.print(" Pitch_d = ");
  Serial.println(desiredPitch);
  Serial.print(" Pitch = ");
  Serial.print(pitch);
}

void printError() {
  Serial.print(" Pitch_error = ");
  Serial.println(errorPitch);
}

void printGains() {
  Serial.print("\t kp: ");
  Serial.print(kp, 6);
  Serial.print(" ki: ");
  Serial.print(ki, 6);
  Serial.print(" kd: ");
  Serial.println(kd, 6);
}

void printPID() {
  Serial.print("\t P-Term = ");
  Serial.print(PTerm);
  Serial.print(" I-Term = ");
  Serial.print(ITerm);
  Serial.print(" D-Term = ");
  Serial.print(DTerm);
  Serial.print(" sumPID = ");
  Serial.print(sumPID);
}

void printOut() {
  Serial.print(" Out = ");
  Serial.println(Out);
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

