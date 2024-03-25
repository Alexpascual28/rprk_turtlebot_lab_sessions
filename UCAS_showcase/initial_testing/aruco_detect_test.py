from ARBPi import *
from TurtleBot import TurtleBot

import time
import math
import threading

import random

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        tb = TurtleBot()

        tb.camera.start_image_acquisition()

        while(True):
            image = tb.camera.get_current_image()

            if image is not None:
                tb.camera.detect_aruco(image)

        # wall_follower = WallFollowerFSM();
        # wall_follower.main()
    except KeyboardInterrupt:
        print('Interrupted!')