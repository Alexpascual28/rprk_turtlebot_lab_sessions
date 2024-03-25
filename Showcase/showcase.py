from ARBPi import *
from TurtleBot import TurtleBot

import time
import math

def main():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)
    
    tb = TurtleBot()

    while(True):
        tb.motors.change_direction("forward");
        tb.motors.set_robot_speed_by_level(7);

        time.sleep(6);

        tb.motors.change_direction("right");
        tb.motors.set_robot_speed_by_level(3);

        time.sleep(3)
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')