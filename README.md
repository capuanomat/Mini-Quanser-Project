# Mini-Quanser-Project
Trying to design a mini, 3 degrees-of-freedom control system similar to the Quanser 3 DOF Helicopter model

![rendered](https://user-images.githubusercontent.com/19838367/29482997-bfe6d6ee-84df-11e7-863a-efeb748eb6a0.JPG)

### HARDWARE WORK:
- Figure out how to add wiring to CAD Files
- Figure out how on earth you will produce several dozens of these frames in metal/acrylic (some thermoplastic polymer maybe? Polystyrene? Try aluminum first, needs   to look like an apple product)
   - The above might work with Injection Moulding! Look into this: https://en.wikipedia.org/wiki/Injection_moulding
- It might be better to use regular AA batteries that you side into the back of the motor arm, rather than a battery pack. Would be a lot more convenient and classy
- Decide between using Arduino Mini or Arduino Micro, or Adafruit Trinket for final product (custom board would be best, but ok for now)


## SOFTWARE WORK
- Write code for second motor
- Add angular acceleration measurements to controls code between angle measurements/adjustments and output
- Add code for Roll (additional PID stuff, might be done at the same time as first point)
- Look into operating the device entirely from MATLAB, must interact corerectly with Arduino Mini


## References
- IMU Tutorial 1: https://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/
- MPU6050 & I2Cdev Libraries: https://github.com/jrowberg/i2cdevlib
- IMU Tutorial 2 (with MATLAB): https://turnsouthat.wordpress.com/2015/07/31/arduino-mpu6050-getting-ready/
- MATLAB IMU scanner: https://github.com/ezgode/Arduino_MPU6050
- PID Library (unused): https://github.com/br3ttb/Arduino-PID-Library
- Secondary MPU6050 Lib (in use): https://github.com/jarzebski/Arduino-MPU6050
- https://github.com/JonHub/Filters
- https://playground.arduino.cc/Code/Filters
- https://en.wikipedia.org/wiki/Integral_windup
