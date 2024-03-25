#
# task5.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 3-4 Task 5
#
# CONTROL THE ROBOT WITH PULSE-WIDTH MODULATION
# Write a program using all the knowledge you have gained that reads keystrokes from the
# serial terminal, and changes the speed of the motors to allow you to drive your robot
# under keyboard control. It is recommended to use a “gaming” style keyboard mapping
# where you can set the direction using the w,a,s,d keys and scale the maximum motor
# speed using the number keys 1...9.
#

from ARBPi import *
import time

REG_RECEIVE_SPEED_DATA = 40 # Arduino receives speed data (Raspberry sends)
REG_RECEIVE_MSG_DRIVE = 41 #// Arduino receives drive control data (Raspberry sends)
REG_SEND_MSG_DRIVE = 42 # Arduino sends drive control data (current state) (Raspberry receives)

def setup():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)

def main_loop(prev_state):
    userinput = input("Enter robot direction (WASD) or speed (0-9):")

    speedInput = ("0", "1", "2", "3", "4", "5", "6", "7", "8", "9")

    if userinput in speedInput:
        register = REG_RECEIVE_SPEED_DATA
        value = int(userinput)
        putRegister(register, value)

    directionInput = {"W": 1, "A": 3, "S": 2, "D": 4,
                      "w": 1, "a": 3, "s": 2, "d": 4}

    if userinput in directionInput:
        register = REG_RECEIVE_MSG_DRIVE
        value = directionInput.get(userinput)
        putRegister(register, value)

    robot_state = getRegister(REG_SEND_MSG_DRIVE)
    
    if((robot_state != prev_state) and (robot_state > 0) and (robot_state <= 5)):
        robot_direction = {1:"moving forward", 2:"moving backward", 3:"moving left", 4:"moving right", 5:"stopped"}
        robot_state = robot_direction.get(robot_state)
        prev_state = robot_state
        print("The robot is " + robot_state)

    time.sleep(0.2)
    return prev_state

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        setup();
        prev_state = 0
        while True:
            prev_state = main_loop(prev_state);

    except KeyboardInterrupt:
        print('interrupted!')
