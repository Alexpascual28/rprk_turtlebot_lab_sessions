#
# task3.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 8 Task 3
#
# IMPLEMENT OBSTACLE DETECTION
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
    
    while True:
        right_ultrasound_distance = tb.ultrasound.get_ultrasound_distance("right")
        left_ultrasound_distance = tb.ultrasound.get_ultrasound_distance("left")
        
        print(f"Right side ultrasound distance: {right_ultrasound_distance} cm")
        print(f"Left side ultrasound distance: {left_ultrasound_distance} cm")

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')
    