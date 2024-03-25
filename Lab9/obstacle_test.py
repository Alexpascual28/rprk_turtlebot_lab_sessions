# -*- coding: utf-8 -*-
"""
Created on Tue May 30 10:53:05 2023

@author: bdh532
"""

from TurtleBot import TurtleBot
import math

def main():
    tb = TurtleBot(estop=False)

    tb.motors.move_continuous(20)
    
    tb.motors.rotate_continuous(math.pi)
    
    tb.motors.move_continuous(20)
    
# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')
