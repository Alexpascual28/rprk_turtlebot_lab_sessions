from ARBPi import *
from TurtleBot import TurtleBot

import time
import math

import random

exit_flag = False

def main():
    state_machine = {
                    0: ["start", (initialize, is_initialized, lambda: True), (1, 0, 0)],
                    1: ["locate", (locate, has_collided, is_located), (3, 2, 1)],
                    2: ["follow wall", (follow_wall, is_back_in_start, has_collided), (4, 3, 2)],
                    3: ["recover", (recover, has_collided, lambda: True), (3, 1, 3)],
                    4: ["finish", (finalize, lambda: True, lambda: True), (4, 4, 4)],
                    }

    state_index = 0

    while exit_flag == False:
        current_state = state_machine[state_index]

        name = current_state[0];
        print("Current state:", name);

        action = current_state[1][0]
        
        action()
        condition1 = current_state[1][1]()
        condition2 = current_state[1][2]()

        state_index = current_state[2][0 if condition1 == True else 1 if condition2 == True else 2]

        time.sleep(0.2)

def initialize():
    print("Initializing robot...")

def is_initialized():
    initialized = bool(random.getrandbits(1))
    print("Initialized") if initialized == True else print("Not initialized")
    return initialized

def locate():
    print("Locating robot...")

def has_collided():
    collided = bool(random.getrandbits(1))
    print("Collision!") if collided == True else print("Recovered!")
    return collided

def is_located():
    located = bool(random.getrandbits(1))
    print("Current location recovered.") if located == True else print("Current location not recovered.")
    return located

def follow_wall():
    print("Following wall and creating map")

def is_back_in_start():
    back_in_start = bool(random.getrandbits(1))
    print("Map explored! Back in start.") if back_in_start == True else print("Exploring map...")
    return back_in_start

def recover():
    print("Recovering from collision...")

def finalize():
    global exit_flag;
    exit_flag = True
    print("Map created. Finalizing program...")

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')