#
# serialComms.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 3-4 Task 5
#
# This example should run in conjunction with the serialComms_receive
# on the Arduino side of things.
# It opens a serial channel to the Arduino, read several registers
# and then writes to a register.
#
# In the C version of this library the char datatype is used to
# limit the data to a single byte, but there is not an equivalent
# datatype in Python, so int is used instead, so take care to not overflow
# 

from ARBPi import *
import time

SERIAL_REGISTER = 0

def setup():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)

    putRegister(5, 0);
    putRegister(6, 127);
    putRegister(7, 128);
    putRegister(8, 255);
    putRegister(9, 256);

def main():
    print("Register 0: ", end="")
    print(getRegister(0))

    print("Register 1: ", end="")
    print(getRegister(1))
    
    print("Register 2: ", end="")
    print(getRegister(2))
    
    print("Register 3: ", end="")
    print(getRegister(3))
    
    print("Register 4: ", end="")
    print(getRegister(4))
    
    time.sleep(0.1)

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        setup();
        while True:
            main();

    except KeyboardInterrupt:
        print('interrupted!')
