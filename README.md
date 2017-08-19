# Mini-Quanser-Project
Trying to design a mini, 3 degrees-of-freedom control system similar to the Quanser 3 DOF Helicopter model

![rendered](https://user-images.githubusercontent.com/19838367/29482997-bfe6d6ee-84df-11e7-863a-efeb748eb6a0.JPG)

### TO DO:
- Finalize CAD stuff (for some reason Pablo thinks the hardware should be **completely** finished (to the finest measurements) before I re-print/assemble it and write more of the code, which is a clear violation of several Agile development principles, notably number 3 and 7, since at this point in time, there is no version of the software that works with the hardware that's been developed over the past two weeks.)
- Add angular accelerations between angles and output
- Add code for Roll (additional PID stuff)
- Look into operating the device entirely from MATLAB, must interact corerectly with Arduino Mini
- Arduino Mini or Adafruit trinket

### DONE:
- Convert Data from IMU into attitudes and angles
- Receive user input through Serial Monitor
- Send signal to motors through ESC and map user input (1000PWM -> 2000PWM)
- Implement basic PID Control
- Make three potentiometers vary the PID gains mapped between 0-1
- (28/06) Redesign and print CAD structure to test 1 degree of freedom: Pitch
- (29/06) Assemble printed parts and try basic testing with BE1306-3100KV motor
- (30/06) Added IMU to frame, finalized connections, mapped controlOutput, managed to make it hold ~horizontal for a few seconds
- (03/07 Week) Implemented PID, filter, and everything else to make model stabilize with 1 motor. Started working on version 1.3 for roll.
- (10/07 Week) Heavily modified version 1.3, three iterations (Designs 1, 2, and 3) can be found atm in Solidworks Parts directory
- (17/07 Week) Finalized version 1.3 Design 3 which is the final hardware version ready for software testing of roll, pitch, and yaw. Also ready for testing with Arduino Mini instead of Arduino

## Needed Hardware Design Improvements (For 1.3)
- Figure out how to add wiring to CAD Files
- Figure out how on earth you will produce several dozens of these frames in metal/acrylic

## References
- IMU Tutorial 1: https://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/
- MPU6050 & I2Cdev Libraries: https://github.com/jrowberg/i2cdevlib
- IMU Tutorial 2 (with MATLAB): https://turnsouthat.wordpress.com/2015/07/31/arduino-mpu6050-getting-ready/
- MATLAB IMU scanner: https://github.com/ezgode/Arduino_MPU6050
- PID Library (unused): https://github.com/br3ttb/Arduino-PID-Library
- Secondary MPU6050 Lib (in use): https://github.com/jarzebski/Arduino-MPU6050

-https://github.com/JonHub/Filters
-https://playground.arduino.cc/Code/Filters
-https://en.wikipedia.org/wiki/Integral_windup
