#
# task1.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 2 Task 1
#
# This example should run in conjunction with the serialComms
# example on the Arduino side of things.
# It opens a serial channel to the Arduino, read several registers
# and then writes to a register.
#
# In the C version of this library the char datatype is used to
# limit the data to a single byte, but there is not an equivalent
# datatype in Python, so int is used instead, so take care to not overflow
# 

from ARBPi import *

# Setup the ARB functions
ARBPiSetup(SERIAL)

# Read the first 5 registers and print their contents
print("Read Test, reading registers 0-9")
print(getRegister(0))
print(getRegister(1))
print(getRegister(2))
print(getRegister(3))
print(getRegister(4))
print(getRegister(5))
print(getRegister(6))
print(getRegister(7))
print(getRegister(8))
print(getRegister(9))
print(getRegister(10))

# Read the 30th register, write into it, then read it back again
print("Write Test, reading register 30")
print(getRegister(30))
print("Writing 48 to register 30")
putRegister(30, 48)
print("Reading back from register 30")
print(getRegister(30))
putRegister(30, 48)
