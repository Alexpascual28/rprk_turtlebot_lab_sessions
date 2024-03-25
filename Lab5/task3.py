#
# task2.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 5 Task 2
#
# MOTION ESTIMATION BASED ON ODOMETRY
# Wheeled robots often sense their distance and speed of movement by measuring the
# revolutions of their wheels using rotary encoders. Although this is rarely accurate over
# long distances, it is useful as a reference when fused with other sensors over short
# distances. Now that you are obtaining the number of counts from the A and B encoder
# outputs using rising edge-triggered interrupts, you need to estimate the overall speed and
# distance covered of the motors
# Make a function that will reliably make the robot move at a desired speed in 
# centimeters per second.
# Implement a move(speed, turn) function in your program
# to allow you to set a desired speed and turning rate based on the “best fit” to the data
# you have gathered. Test and retain this program with both linear movement and turning
# functionality for the next tasks.
#

from ARBPi import *
from TurtleBot import TurtleBot

import time
import math

def main():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)
    
    tb = TurtleBot()
    
    tb.motors.move(20)
    
    time.sleep(1)
    
    tb.motors.rotate(math.pi/2)
    
    time.sleep(1)
    
    tb.motors.rotate(-math.pi/2)
    
    time.sleep(1)
    
    tb.motors.move(-20)
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')
    