# -*- coding: utf-8 -*-
"""
Created on Wed May 24 14:56:11 2023

@author: Alejandro Pascual San Roman (bdh532)
@organisation: Department of Physics, Engineering and Technology. University of York
@title: Turtlebot class

"""

from ARBPi import *
import math
import time

class TurtleBot:
    def __init__(self):
        self.initial_pose = [0, 0, 0] #  x, y, w(orientation)
        self.motors = self.Motors(self, self.initial_pose)
        
    # Reads two values from separate registers and turns them into a 16bit signed integer
    def read_16bit_number(self, register1, register2):
        first_part = getRegister(register2)
        second_part = getRegister(register1)
        
        result = (first_part << 7) + second_part # Shifts first value by seven bits and add the second value
        return result

    # Reads two values from separate registers and turns them into a 2-fractional decimal number
    def read_fractional_number(self, register1, register2):
        wholePart = getRegister(register1)
        fractPart = getRegister(register2)
        
        resultFrac = wholePart + fractPart/100
        return resultFrac
            
    class Motors:
        def __init__(self, turtle_bot, initial_pose):
            self.distance_between_wheels = 21
            
            self.distance_step = 2 # The robot will move x centimetres per step
            self.distance_margin = 1 # The robot will reach its destination if it is within x centimetres of target distance
            self.linear_speed = 7 # The robot will move at x centimetres per second initially, when moving forwards or backwards
            
            self.rotational_step = math.pi/8 # The robot will rotate x radians per rotation
            self.rotational_margin = math.pi/32 # The robot will reach its target orientation if it is within x radians
            self.rotation_speed = 5 # The wheels will move x centimetres per second, when the robot is rotating
            
            self.turtle_bot = turtle_bot
            self.current_pose = initial_pose # x, y, w
            
            # Serial registers
            self.REG_RECEIVE_DIR_MOTOR_A = 30 # Motor A receive direction
            self.REG_RECEIVE_PWM_MOTOR_A = 31 # Motor A receive PWM
            self.REG_SEND_DATA_MOTOR_A = 32 # Motor A send data (current direction and speed)

            self.REG_RECEIVE_DIR_MOTOR_B = 33 # Motor B receive direction
            self.REG_RECEIVE_PWM_MOTOR_B = 34 # Motor B receive PWM
            self.REG_SEND_DATA_MOTOR_B = 35 # Motor B send data (current direction and speed)

            self.REG_SEND_DATA_ENCODER_A_1 = 36 # Encoder A send messages (Rasperry receives)
            self.REG_SEND_DATA_ENCODER_A_2 = 37 # Encoder A send data (Raspberry recieves)

            self.REG_SEND_DATA_ENCODER_B_1 = 38 # Encoder B send messages (Raspberry receives)
            self.REG_SEND_DATA_ENCODER_B_2 = 39 # Encoder B send data (Raspberry receives)

            self.REG_RECEIVE_SPEED_DATA = 40 # Arduino receives speed data (Raspberry sends)
            self.REG_RECEIVE_MSG_DRIVE = 41 # Arduino receives drive control data (Raspberry sends)
            self.REG_SEND_MSG_DRIVE = 42 # Arduino sends drive control data (current state) (Raspberry receives)

            self.REG_SEND_DISTANCE_A = 43 # Calculated distance in wheel A
            self.REG_SEND_DISTANCE_A_DEC = 44 # Calculated distance in wheel A (decimal part)

            self.REG_SEND_DISTANCE_B = 45 # Calculated distance in wheel B
            self.REG_SEND_DISTANCE_B_DEC = 46 # Calculated distance in wheel B (decimal part)

            self.REG_SEND_SPEED_A = 47 # Calculated speed of wheel A
            self.REG_SEND_SPEED_A_DEC = 48 # Calculated speed of wheel A (decimal part)

            self.REG_SEND_SPEED_B = 49 # Calculated speed of wheel B
            self.REG_SEND_SPEED_B_DEC = 50 # Calculated speed of wheel B (decimal part)
            
        # Changes the robot direction based on global direction commands
        def change_direction(self, direction):
            possible_directions = {"forward": 1, "backward": 2, "left": 3, "right": 4, "stop": 5}
            
            if direction in possible_directions:
                # Send data
                putRegister(self.REG_RECEIVE_MSG_DRIVE, possible_directions.get(direction))
                print("Moving " + direction)
            else:
                print("Incorrect direction.")
                
        # Changes the robot speed based on the given speed level
        def set_robot_speed_by_level(self, speed_level):
            # If the user input is 0-9
            if speed_level in range(10):
                # Send data
                putRegister(self.REG_RECEIVE_SPEED_DATA, speed_level)
                print("Setting speed level to " + str(speed_level))
            else:
                print("Incorrect speed level.")
                
        # Changes the wheel speed based on a given speed in centimetres per second
        def set_wheel_speed(self, wheel, speed_cm_s):
            # Identifies which registers to send to based on input
            possible_wheels = {"A": self.REG_RECEIVE_PWM_MOTOR_A,
                               "B": self.REG_RECEIVE_PWM_MOTOR_B}
            if wheel in possible_wheels:
                # Calculates PWM signal from given metric speed in centimetres per second (cm/s)
                average_pwm_signal = 126 # Measured average PWM
                average_speed_cm_s = 7.3795 # Measured average speed
                speed_pwm = int((average_pwm_signal/average_speed_cm_s)*speed_cm_s); # Converts to int
                
                # Adjusts value
                if speed_pwm > 255:
                    speed_pwm = 255
                elif speed_pwm < 0:
                    speed_pwm = 0
                
                # Sends value to the appropiate register
                putRegister(possible_wheels.get(wheel), speed_pwm)
                print("Setting speed of wheel " + wheel + " to " + str(speed_cm_s) + " cm/s (" + str(speed_pwm) + " PWM)")
            else:
                print("Incorrect wheel input.")
        
        # Changes the wheel direction based on a given direction command
        def set_wheel_direction(self, wheel, direction):
            # Identifies which registers to send to based on input
            possible_wheels = {"A": self.REG_RECEIVE_DIR_MOTOR_A,
                               "B": self.REG_RECEIVE_DIR_MOTOR_B}
            
            possible_directions = {"CW": 0, "CCW": 1}
            
            if wheel in possible_wheels:
                if direction in possible_directions:
                    # Sends appropiate value to the appropiate register
                    putRegister(possible_wheels.get(wheel), possible_directions.get(direction))
                    print("Setting wheel " + wheel + " direction to " + direction)
                else:
                    print("Incorrect wheel direction")
            else:
                print("Incorrect wheel input.")
        
        # Sets speed of both wheels simultaneously
        def set_robot_speed(self, speed):
            self.set_wheel_speed("A", speed);
            self.set_wheel_speed("B", speed);
            
        # Reads current steps from the encoder registers
        def get_current_steps(self, encoder):
            possible_encoders = {"A": [self.REG_SEND_DATA_ENCODER_A_1, self.REG_SEND_DATA_ENCODER_A_2],
                                 "B": [self.REG_SEND_DATA_ENCODER_B_1, self.REG_SEND_DATA_ENCODER_B_2]}
            
            if encoder in possible_encoders:
                r1, r2 = possible_encoders.get(encoder)
                steps = self.turtle_bot.read_16bit_number(r1, r2)
                return steps
            else:
                print("Incorrect encoder.")
        
        # Reads current travelled distance from the encoder registers
        def get_current_distance(self, encoder):
            possible_encoders = {"A": [self.REG_SEND_DISTANCE_A, self.REG_SEND_DISTANCE_A_DEC],
                                 "B": [self.REG_SEND_DISTANCE_B, self.REG_SEND_DISTANCE_B_DEC]}
            
            if encoder in possible_encoders:
                r1, r2 = possible_encoders.get(encoder)
                distance = self.turtle_bot.read_fractional_number(r1, r2)
                return distance
            else:
                print("Incorrect encoder.")
        
        # Reads current robot speed from the encoder registers
        def get_current_speed(self, encoder):
            possible_encoders = {"A": [self.REG_SEND_SPEED_A, self.REG_SEND_SPEED_A_DEC],
                                 "B": [self.REG_SEND_SPEED_B, self.REG_SEND_SPEED_B_DEC]}
            
            if encoder in possible_encoders:
                r1, r2 = possible_encoders.get(encoder)
                speed = self.turtle_bot.read_fractional_number(r1, r2)
                return speed
            else:
                print("Incorrect encoder.")
            
        def move(self, distance):
            print("Moving robot by " + str(distance) + " centimetres")
            if distance != 0:
                # Set robot direction based on input
                if distance > 0:
                    self.change_direction("forward")
                    self.set_robot_speed(0) # Start by turning off the motors
                elif distance < 0:
                    self.change_direction("backward") # If the distance is negative, move backwards
                    self.set_robot_speed(0) # Start by turning off the motors
                else:
                    print("Incorrect distance input")
                    return
                
                # Store distance travelled and a flag to check if the distance has been reached
                distance_travelled = 0
                is_distance_reached = False
                
                # While the distance has not been reached
                while is_distance_reached == False:
                    # Change the speed to the parameter linear speed
                    self.set_robot_speed(self.linear_speed)
                    
                    # Move until the step distance has been reached
                    time.sleep(self.distance_step/self.linear_speed)
                    
                    # Stop robot for some time, to allow encoder readings to reach
                    self.set_robot_speed(0)
                    time.sleep(0.1)
                    
                    # Calculate distance change and total distance travelled
                    distance_change, _ = self.calculate_current_pose()
                    distance_travelled += distance_change
                    print("Distance travelled: " + str(distance_travelled) + " cm")
                    
                    # If the diatnce travelled is within the margin, stop the loop
                    if (distance_travelled >= distance - self.distance_margin) and (distance_travelled <= distance + self.distance_margin):
                        print("Robot succesfully moved to " + str(self.current_pose))
                        is_distance_reached = True
                        break
                    
                    # If the robot has overestimated the position, move in the oposite direction to compensate
                    elif distance_travelled > distance + self.distance_margin:
                        self.change_direction("backward")
                    elif distance_travelled < distance - self.distance_margin:
                        self.change_direction("forward")
                
            else:
                self.change_direction("stop")
                self.set_robot_speed(0)
            
        def rotate(self, angle):
            print("Rotating robot by " + str(angle) + " radians")
            if angle != 0:
                # Set robot direction based on input
                if angle > 0:
                    self.change_direction("left")
                    self.set_robot_speed(0) # Start by turning off the motors
                elif angle < 0:
                    self.change_direction("right") # If the angle is negative, move to the right
                    self.set_robot_speed(0) # Start by turning off the motors
                else:
                    print("Incorrect angle input")
                    return
                
                # Store angle travelled and a flag to check if the angle has been reached
                angle_travelled = 0
                is_angle_reached = False
                
                # While the angle has not been reached
                while is_angle_reached == False:
                    # Change the speed to the rotation speed parameter value
                    self.set_robot_speed(self.rotation_speed)
                    
                    # Move until the step distance has been reached
                    time.sleep(self.rotational_step/self.rotation_speed)
                    
                    # Stop robot for some time, to allow encoder readings to reach
                    self.set_robot_speed(0)
                    time.sleep(0.1)
                    
                    # Calculate distance change and total distance travelled
                    _, angle_change = self.calculate_current_pose()
                    angle_travelled += angle_change
                    print("Angle travelled: " + str(angle_travelled) + " rad")
                    
                    # If the angle travelled is within the margin, stop the loop
                    if (angle_travelled >= angle - self.rotational_margin) and (angle_travelled <= angle + self.rotational_margin):
                        print("Robot succesfully rotated by " + str(angle) + " radians")
                        print("New robot pose: " + str(self.current_pose))
                        is_angle_reached = True
                        break
                    
                    # If the robot has overestimated the position, move in the oposite direction to compensate
                    elif angle_travelled > angle + self.rotational_margin:
                        self.change_direction("right")
                    elif angle_travelled < angle - self.rotational_margin:
                        self.change_direction("left")
                
            else:
                self.change_direction("stop")
                self.set_robot_speed(0)
        
        # Calculates the next pose based on calculated travelled distance of each wheel and stores it in self.current_pose
        def calculate_current_pose(self):
            distance_a = self.get_current_distance("A")
            distance_b = self.get_current_distance("B")
            
            # With my own calculations. This is all geometrically correct, but may cause problems if the distances are similar, as the radius value would approach infinity.
            # radius_b = (distance_b * self.distance_between_wheels)/(distance_a - distance_b) 
            # angle_change = distance_b/radius_b
            # radius = radius_b + self.distance_between_wheels/2
            # distance_change = 2 * radius * math.sin(angle_change/2)
            
            # Calculations based on https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
            angle_change = (distance_b - distance_a)/self.distance_between_wheels
            distance_change = (distance_a + distance_b)/2 # Since the movement is small, it assumes the circular distance is equal to the linear distance
            
            self.current_pose[0] = self.current_pose[0] + distance_change * math.cos(self.current_pose[2]+ (angle_change/2))
            self.current_pose[1] = self.current_pose[1] + distance_change * math.sin(self.current_pose[2]+ (angle_change/2))
            self.current_pose[2] = self.current_pose[2] + angle_change
            
            return distance_change, angle_change
            
    class Camera:
        def __init__(self):
            pass
            
    class Ultrasonic:
        def __init__(self):
            pass
            
    class Joystick:
        def __init__(self):
            pass
            
    class InfraredSensor:
        def __init__(self):
            pass
