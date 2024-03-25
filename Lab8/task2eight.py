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
    
    tb = TurtleBot()
    
    radius = 5
    vertical_distance = 10
    
    distance = math.sqrt(vertical_distance^2 + (2*radius)^2)
    diagonal_angle = math.atan(vertical_distance/(2*radius))
    
    section_distance = tb.motors.distance_step
    
    perimeter = 2 * math.pi * radius
    number_of_sections = perimeter / section_distance
    
    section_angle = (2 * math.pi)/number_of_sections
    
    while True:
        while not(tb.motors.current_pose[2] >= math.pi):
            tb.motors.move_with_encoders(section_distance)
            tb.motors.rotate_with_encoders(section_angle/2) # Maybe section_angle/2?
            
        tb.motors.rotate_with_encoders(diagonal_angle)
        tb.motors.move_with_encoders(distance)
        tb.motors.rotate_with_encoders(-diagonal_angle)
        
        while not(tb.motors.current_pose[2] <= 0):
            tb.motors.move_with_encoders(section_distance)
            tb.motors.rotate_with_encoders(-(section_angle/2)) # Maybe section_angle/2?
            
        tb.motors.rotate_with_encoders(-diagonal_angle)
        tb.motors.move_with_encoders(distance)
        tb.motors.rotate_with_encoders(diagonal_angle)
        

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')
    