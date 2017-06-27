# Mini-Quanser-Project
Trying to design a mini, 3 degree control system similar to the Quanser 3 DOF Helicopter model

### TO DO:
- Redesign and print CAD structure to test 1 degree of freedom: Pitch
-- Assemble it

### DONE:
- Convert Data from IMU into attitudes and angles
- Receive user input through Serial Monitor
- Send signal to motors through ESC and map user input (1000PWM -> 2000PWM)
- Implement basic PID Control
- Make three potentiometers vary the PID gains mapped between 0-1
