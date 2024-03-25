# -*- coding: utf-8 -*-
"""
Created on Tue May 30 12:59:47 2023

@author: bdh532
"""

from TurtleBot import TurtleBot
import time

def main():    
    tb = TurtleBot()
    
    directions = ["forward", "backward", "left", "right"]
    
    for direction in directions:          
        tb.motors.change_direction(direction)
        
        for speed_level in range(1, 10):
            # Resets encoders
            tb.motors.reset_encoder("A")
            tb.motors.reset_encoder("B")
            
            # Get initial time
            initial_time = time.time()
            
            # Change speed_level
            tb.motors.set_robot_speed_by_level(speed_level)
            
            # Move for 0.2 seconds
            time.sleep(0.2)
            
            # Get current time
            current_time = time.time()
            
            # Calculate time difference
            time_difference_s = current_time - initial_time
            print(f"Time Difference: {time_difference_s}")
            
            # Read steps
            stepsA = tb.motors.get_current_steps("A")
            stepsB = tb.motors.get_current_steps("B")
            print(f"Steps A: {stepsA}")
            print(f"Steps B: {stepsB}")
            
            # Calculate distances directly from steps
            distanceA = tb.motors.calculate_distance_cm(stepsA)
            distanceB = tb.motors.calculate_distance_cm(stepsB)
            print(f"Distance A: {distanceA}")
            print(f"Distance A: {distanceB}")
            
            # Read speeds
            speedA = tb.motors.calculate_speed_cm_s(distanceA, time_difference_s)
            speedB = tb.motors.calculate_speed_cm_s(distanceA, time_difference_s)
            print(f"Speed A: {speedA}")
            print(f"Speed B: {speedB}\n")
            
    # Stop robot
    tb.motors.set_robot_speed_by_level(0)
        
    
# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted!')

