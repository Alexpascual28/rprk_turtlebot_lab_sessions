#
# task2.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 2 Task 2
#
# This example should run in conjunction with the I2CMux_serial
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

IR_SERIAL_REGISTER = 0

def read_IR_sensor(ir_serial_register):
    distance = getRegister(ir_serial_register) # Returns distance in cm
    return distance

def setup():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)

def main_loop():
    # Gets distance written to the IR register, read from the IR sensor connected to the Arduino via I2C MUX
    distance = read_IR_sensor(IR_SERIAL_REGISTER)
    
    print("The distance read from the IR sensor is " + str(distance) + " cm")

    time.sleep(0.2)

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        setup();

        while True:
            main_loop();

    except KeyboardInterrupt:
        print('interrupted!')

