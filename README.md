
# Raspberry Pi Pico python code to read and display orientation 

The four python modules are classes used to access the Waveshare MPU and
OLED display.   

The 4 python modules are:
### 1. PicoMain.py
The main module that calls the other 3 classes.
### 2. PicoMPU.py
Module that retrieves data from the MPU9250 module and includes calibration code.
### 3. PicoLCD.py 
Module that sets up the Waveshare Pico-1.14-LCD (version B). The display has 240 x 135 multi-color pixels.
### 4. PicoJoy.py
Module that reads the joystick and returns button A and B values

### Testing

This code has been tested on a Linux computer running the Thonny interface.   
This has not been tested on an Arduino.

To have the code run automatically at power on, copy the PicoMain.py to main.py on the Raspberry Pi Pico.
Copy the other files to the Pico with no name changes.   The code automatically then starts running upon power-on
### Hardware
The Waveshare hardware is described on a Wiki:
[] https://www.waveshare.com/wiki/Pico-10DOF-IMU

The parts were ordered as a part of a kit on Amazon:
[]https://www.amazon.com/Raspberry-Evaluation-Pre-soldered-1-14inch-Breadboard/dp/B08YP3J5R6

Glen Langston
August 7, 2023
