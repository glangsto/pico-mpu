"""
Class to setup the joystick for the Waveshare Pico 1.14 LCD hat version B,
which includes a Joy Stick and two buttons, A and B.
Returns key presses and joystick motion by Pico Screen.
Separated from LCD classe, Except when run
as a main program for testing.

This class is intended to allow interaction the buttons
for calibration of the MPU 9250 sensor.

Glen Langston, National Science Foundation
August 7, 2023
"""
from machine import Pin
import time

class PicoJoy:

    ## Constructor
    #  @param [in] address MPU-9250 I2C slave address default:0x68
    def __init__(self, device="Pico1_14"):
        # define key stroke indicies
        self.A = 0
        self.B = 1
        self.UP = 2
        self.DOWN = 5    # setup Buttons
        self.CNTL = 3
        self.LEFT = 4
        self.RIGHT = 6
        self.keys = [0, 0, 0, 0, 0, 0, 0]
       
    def keyA(self,verbose=False):
        """
        keys A and B are pulled to 1.
        If zero then keys are pressed
        """
        valueA = Pin(15,Pin.IN,Pin.PULL_UP).value()
        if verbose and valueA == 0:
            print("A")
        if valueA == 0:
            self.keys[0] = 1
            return True
        else:
            self.keys[0] = 0 
            return False
        
    def keyB(self, verbose=False):
        """
        keys A and B are pulled to 1.
        If zero then keys are pressed
        """
        valueB = Pin(17,Pin.IN,Pin.PULL_UP).value()
        if verbose and valueB == 0:
            print("B")
        if valueB == 0:
            self.keys[1] = 1
            return True
        else:
            self.keys[1] = 0
            return False
    
    # the following definitions return 0 if key is pressed
    def key2(self):
        return Pin(2 ,Pin.IN,Pin.PULL_UP).value()
    #上
    def key3(self):
        return Pin(3 ,Pin.IN,Pin.PULL_UP).value() #中

    def key4(self):
        return Pin(16 ,Pin.IN,Pin.PULL_UP).value()#左
    
    def key5(self):
        return Pin(18 ,Pin.IN,Pin.PULL_UP).value() #下
    
    def key6(self):
        return Pin(20 ,Pin.IN,Pin.PULL_UP).value() #右

        
    def joystick(self, verbose=False):
        """
        Check all joystick directions and report
        Optionally the verbose flag may be used for debugging.
        Returns true if a joystick sensor value is high,
        plus a 7 value array array keys[].  Off is 0, On is 1.
        """
        keys = [0, 0, 0, 0, 0, 0, 0]
        if self.key2() == 0:#上
            keys[self.UP] = 1
            if verbose:
                print("UP")
            
        if self.key3() == 0:#中
            keys[self.CNTL] = 1
            if verbose:
                print("CNTL")
            
        if self.key4() == 0:#左
            keys[self.LEFT] = 1
            if verbose:
                print("LEFT")

        if self.key5() == 0:#下
            keys[self.DOWN] = 1
            if verbose:
                print("DOWN")
            
        if self.key6() == 0:#右
            if verbose:
                print("RIGHT")
            keys[self.RIGHT] = 1
            
        if keys[0] + keys[1] + keys[2] + keys[3] + keys[4] + \
           keys[5] + keys[6] > 0:
            keyPushed = True
        else :
            keyPushed = False
        
        # keep track of key values
        self.keys = keys
        return keyPushed, keys
        # end of joystick()
        
    def wait(self, verbose=False):
        """
        Wait for any joystick key before returning
        """
        keyPushed = False
        while not keyPushed:
            keyPushed, keys = self.joystick(verbose=verbose);
            
        return
    
if __name__ == '__main__':
    """
    The main function is not normally used, except for
    testing.   This module does depend on the LCD class,
    for indication of button presses.
    """
    import PicoLCD

    # prepare to display on the oled screen
    oled = PicoLCD.PicoLCD()
    # This module assumes a 240x135 pixel multi-colored screen
    oled.fill(0xFFFF)   # white screen
    oled.show()

    joy = PicoJoy()
    print("Running PicoJoy Test")
    
    while True:
        
        if joy.keyA(verbose=True):
            oled.fill_rect(208,12,20,20,oled.red)
        else :
            oled.fill_rect(208,12,20,20,oled.white)
            oled.rect(208,12,20,20,oled.red)
            
        if joy.keyB(verbose=True):
            oled.fill_rect(208,103,20,20,oled.red)
        else:
            oled.fill_rect(208,103,20,20,oled.white)
            oled.rect(208,103,20,20,oled.red)
        
        keyPushed, keys = joy.joystick(verbose=True)
        if keyPushed:
            oled.fill_rect(12,12,20,20,oled.red)
        else:
            oled.fill_rect(12,12,20,20,oled.white)
            oled.rect(12,12,20,20,oled.red)
         
        oled.show()
        time.sleep(.1)
        # end of main
