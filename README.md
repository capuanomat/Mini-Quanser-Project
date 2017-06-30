# Mini-Quanser-Project
Trying to design a mini, 3 degrees-of-freedom control system similar to the Quanser 3 DOF Helicopter model

### TO DO:
- (29/06) Assemble printed parts and try basic testing with BE1306-3100KV motor

### DONE:
- Convert Data from IMU into attitudes and angles
- Receive user input through Serial Monitor
- Send signal to motors through ESC and map user input (1000PWM -> 2000PWM)
- Implement basic PID Control
- Make three potentiometers vary the PID gains mapped between 0-1
- (28/06) Redesign and print CAD structure to test 1 degree of freedom: Pitch

## Needed Hardware Design Improvements (After 2.0)
- Increase spacing between motor arm and ball bearing frame to let wires fit through without adding friction
- Include small hole/dent between screw holes on motor mount to fit middle masher and nail thing (what you soldered into 3D print)
- Increase spacing between ball bearing mount arms to fit the motor arm more easily (adjust motor arm spacing as needed)
- Design second motor mount for two motors to test roll

## References
- IMU Tutorial 1: https://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/
- MPU6050 * I2Cdev Libraries: https://github.com/jrowberg/i2cdevlib

- IMU Tutorial 2 (with MATLAB): https://turnsouthat.wordpress.com/2015/07/31/arduino-mpu6050-getting-ready/
-- MATLAB IMU scanner: https://github.com/ezgode/Arduino_MPU6050

- PID Library (Unused): https://github.com/br3ttb/Arduino-PID-Library
- Secondary MPU6050 Lib (in use): https://github.com/jarzebski/Arduino-MPU6050
