#
# task2.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 8 Task 2
#
#

from ARBPi import *
from TurtleBot import TurtleBot

import time
import math

def main():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)
    
    tb = TurtleBot(estop=True)
    
    tb.move(100)
    
# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')
    