#
# task2p1.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 5 Task 2 Part 1
#
# READ IN THE ENCODER COUNTS - WASD CONTROL - USING TURTLEBOT CLASS
#

from ARBPi import *
import time
from TurtleBot import TurtleBot

def main():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)
    
    # Sets initial values for variables
    prev_stepsA = 0
    prev_stepsB = 0
    prev_distanceA = 0
    prev_distanceB = 0
    prev_speedA = 0
    prev_speedB = 0
    
    tb = TurtleBot()
    
    while True:
        # Read user input
        userinput = input("Enter robot direction (WASD) or speed (0-9):")
        speedInput = ("0", "1", "2", "3", "4", "5", "6", "7", "8", "9")

        # If the user input is 0-9
        if userinput in speedInput:
            tb.motors.set_robot_speed_by_level(int(userinput))

        directionInput = {"W": "forward", "A": "left", "S": "backward", "D": "right",
                          "w": "forward", "a": "left", "s": "backward", "d": "right"}

        # If the user input is WASD
        if userinput in directionInput:
            direction = directionInput.get(userinput) # Get associated signal in directionInput
            tb.motors.change_direction(direction)
            
        # Read steps
        stepsA = tb.motors.get_current_steps("A")
        stepsB = tb.motors.get_current_steps("B")
        
        # Read distances
        distanceA = tb.motors.get_current_distance("A")
        distanceB = tb.motors.get_current_distance("B")
        
        # Read speeds
        speedA = tb.motors.get_current_speed("A")
        speedB = tb.motors.get_current_speed("B")
        
        # If any of these values has changed, print them (Motor A)
        if(stepsA != prev_stepsA):
            print("A steps: " + str(stepsA))
            prev_stepsA = stepsA
        if(distanceA != prev_distanceA):
            print("A distance: " + str(distanceA) + " cm")
            prev_distanceA = distanceA
        if(speedA != prev_speedA):
            print("A speed: " + str(speedA) + " cm/s")
            prev_speedA = speedA
        
        # If any of these values has changed, print them (Motor B)
        if(stepsB != prev_stepsB):
            print("B steps: " + str(stepsB))
            prev_stepsB = stepsB
        if(distanceB != prev_distanceB):
            print("B distance: " + str(distanceB) + " cm")
            prev_distanceB = distanceB
        if(speedB != prev_speedB):
            print("B speed: " + str(speedB) + " cm/s")
            prev_speedB = speedB
            
        time.sleep(0.2)

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')
    