# -*- coding: utf-8 -*-
"""
Created on Wed May 24 14:56:11 2023

@author: Alejandro Pascual San Roman (bdh532)
@organisation: Department of Physics, Engineering and Technology. University of York
@title: Turtlebot class

@version: v2

"""

from ARBPi import *
import math
import time
import numpy as np

# Import the necessary packages
import picamera
import picamera.array
import cv2
from cv2 import aruco

class TurtleBot:
    def __init__(self, estop=True):
        self.initial_pose = [0, 0, 0] #  x, y, w(orientation)
        self.estop = estop
        
        self.motors = self.Motors(self, self.initial_pose)
        self.camera = self.Camera()
        self.infrared = self.InfraredSensor()
        self.ultrasound = self.Ultrasonic()
        self.joystick = self.Joystick()
        
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
    
    def move(self, distance):
        print(f"Moving robot by {distance} centimetres")
        if distance != 0:
            current_direction = ""
            
            # Set robot direction based on input
            if distance > 0:
                current_direction = "forward"
                self.motors.change_direction(current_direction)
                self.motors.set_robot_speed(0) # Start by turning off the motors
            elif distance < 0:
                current_direction = "backward"
                self.motors.change_direction(current_direction) # If the distance is negative, move backwards
                self.motors.set_robot_speed(0) # Start by turning off the motors
            else:
                print("Incorrect distance input")
                return
            
            # Store distance travelled and a flag to check if the distance has been reached
            distance_travelled = 0
            is_distance_reached = False
            estop_activated = False
            
            if self.estop == True:
                previous_distance_to_obstacle = 11 # Initialize at a value that wont cause an estop
            
            # While the distance has not been reached
            while is_distance_reached == False and estop_activated == False:
                # Change the speed to the parameter linear speed
                self.motors.set_robot_speed(self.motors.linear_speed)
                
                # Move until the step distance has been reached
                time.sleep(self.motors.distance_step/self.motors.linear_speed)
                
                # Stop robot for some time, to allow encoder readings to reach
                self.motors.set_robot_speed(0)
                time.sleep(0.001)
                
                # Calculate distance change and total distance travelled
                distance_change, _ = self.motors.calculate_current_pose()
                distance_travelled += distance_change
                print(f"Distance travelled: {distance_travelled} cm")
                
                # If the diatnce travelled is within the margin, stop the loop
                if (distance_travelled >= distance - self.motors.distance_margin) and (distance_travelled <= distance + self.motors.distance_margin):
                    print(f"Robot succesfully moved to {self.motors.current_pose}")
                    is_distance_reached = True
                    break
                
                # If the robot is too close to an object, stop
                if self.estop == True:
                    distance_to_obstacle = self.infrared.get_infrared_distance()
                    
                    if distance_to_obstacle < 4 or (distance_to_obstacle == 63 and previous_distance_to_obstacle < 10):
                        print("Infrared E-stop")
                        self.motors.change_direction("stop")
                        self.motors.set_robot_speed(0)
                        estop_activated = True
                        break
                    
                    previous_distance_to_obstacle = distance_to_obstacle
                
                # If the robot has overestimated the position, move in the oposite direction to compensate
                if distance_travelled > distance + self.motors.distance_margin:
                    if current_direction == "forward":
                        current_direction = "backward"
                        self.motors.change_direction(current_direction)
                elif distance_travelled < distance - self.motors.distance_margin:
                    if current_direction == "backward":
                        current_direction = "forward"
                        self.motors.change_direction(current_direction)
            
        else:
            self.motors.change_direction("stop")
            self.motors.set_robot_speed(0)
        
    def rotate(self, angle):
        print(f"Rotating robot by {angle} radians")
        if angle != 0:
            current_direction = ""
            
            # Set robot direction based on input
            if angle > 0:
                current_direction = "left"
                self.motors.change_direction("left")
                self.motors.set_robot_speed(0) # Start by turning off the motors
            elif angle < 0:
                current_direction = "right"
                self.motors.change_direction("right") # If the angle is negative, move to the right
                self.motors.set_robot_speed(0) # Start by turning off the motors
            else:
                print("Incorrect angle input")
                return
            
            # Store angle travelled and a flag to check if the angle has been reached
            angle_travelled = 0
            is_angle_reached = False
            estop_activated = False
            
            # While the angle has not been reached
            while is_angle_reached == False and estop_activated == False:
                # Change the speed to the rotation speed parameter value
                self.motors.set_robot_speed(self.motors.rotation_speed)
                
                # Move until the step distance has been reached
                time.sleep(self.motors.rotational_step/self.motors.rotation_speed)
                
                # Stop robot for some time, to allow encoder readings to reach
                self.motors.set_robot_speed(0)
                time.sleep(0.001)
                
                # Calculate distance change and total distance travelled
                _, angle_change = self.motors.calculate_current_pose()
                angle_travelled += angle_change
                print(f"Angle travelled: {angle_travelled} rad")
                
                # If the angle travelled is within the margin, stop the loop
                if (angle_travelled >= angle - self.motors.rotational_margin) and (angle_travelled <= angle + self.motors.rotational_margin):
                    print(f"Robot succesfully rotated by {angle} radians")
                    print(f"New robot pose: {self.motors.current_pose}")
                    is_angle_reached = True
                    break
                
                if self.estop == True:
                    distance_to_obstacle_right = self.ultrasound.get_ultrasound_distance("right")
                    distance_to_obstacle_left = self.ultrasound.get_ultrasound_distance("left")
                    
                    if distance_to_obstacle_right < 4 or distance_to_obstacle_left < 4:
                        print("Ultrasound E-stop")
                        self.motors.change_direction("stop")
                        self.motors.set_robot_speed(0)
                        estop_activated = True
                        break
                
                # If the robot has overestimated the position, move in the oposite direction to compensate
                if angle_travelled > angle + self.motors.rotational_margin:
                    if current_direction == "left":
                        current_direction = "right"
                        self.motors.change_direction("right")
                elif angle_travelled < angle - self.motors.rotational_margin:
                    if current_direction == "right":
                        current_direction = "left"
                        self.motors.change_direction("left")
            
        else:
            self.motors.change_direction("stop")
            self.motors.set_robot_speed(0)
            
    class Motors:
        def __init__(self, turtle_bot, initial_pose):
            self.distance_between_wheels = 21
            
            self.distance_step = 0.4 # The robot will move x centimetres per step
            self.distance_margin = 0.2 # The robot will reach its destination if it is within x centimetres of target distance
            self.linear_speed = 5 # The robot will move at x centimetres per second initially, when moving forwards or backwards
            
            self.rotational_step = math.pi/16 # The robot will rotate x radians per rotation
            self.rotational_margin = math.pi/128 # The robot will reach its target orientation if it is within x radians
            self.rotation_speed = 10 # The wheels will move x centimetres per second, when the robot is rotating
            
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
            
            self.REG_RECEIVE_RESET_ENCODER_A = 51 # Resets absolute steps of encoder A
            self.REG_RECEIVE_RESET_ENCODER_B = 52 # Resets absolute steps of encoder B
            
        # Changes the robot direction based on global direction commands
        def change_direction(self, direction):
            possible_directions = {"forward": 1, "backward": 2, "left": 3, "right": 4, "stop": 5}
            
            if direction in possible_directions:
                # Send data
                putRegister(self.REG_RECEIVE_MSG_DRIVE, possible_directions.get(direction))
                print(f"Moving {direction}")
            else:
                print("Incorrect direction.")
                
        # Changes the robot speed based on the given speed level
        def set_robot_speed_by_level(self, speed_level):
            # If the user input is 0-9
            if speed_level in range(10):
                # Send data
                putRegister(self.REG_RECEIVE_SPEED_DATA, speed_level)
                print(f"Setting speed level to {speed_level}")
            else:
                print("Incorrect speed level.")
                
        # Changes the wheel speed based on a given speed in centimetres per second
        def set_wheel_speed(self, wheel, speed_cm_s):
            # Identifies which registers to send to based on input
            possible_wheels = {"A": [self.REG_RECEIVE_PWM_MOTOR_A, 140, 8.356667],
                               "B": [self.REG_RECEIVE_PWM_MOTOR_B, 140, 8.042222]}
            if wheel in possible_wheels:
                # Calculates PWM signal from given metric speed in centimetres per second (cm/s)
                average_pwm_signal = possible_wheels.get(wheel)[1] # Measured average PWM
                average_speed_cm_s = possible_wheels.get(wheel)[2] # Measured average speed
                speed_pwm = int((average_pwm_signal/average_speed_cm_s)*speed_cm_s); # Converts to int
                
                # Adjusts value
                if speed_pwm > 255:
                    speed_pwm = 255
                elif speed_pwm < 0:
                    speed_pwm = 0
                
                # Sends value to the appropiate register
                putRegister(possible_wheels.get(wheel)[0], speed_pwm)
                # print(f"Setting speed of wheel {wheel} to {speed_cm_s} cm/s ({speed_pwm} PWM)")
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
                    print(f"Setting wheel {wheel} direction to {direction}")
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
                
        # Resets encoders
        def reset_encoder(self, encoder):
            possible_encoders = {"A": self.REG_RECEIVE_RESET_ENCODER_A,
                                 "B": self.REG_RECEIVE_RESET_ENCODER_B}
            
            if encoder in possible_encoders:
                putRegister(possible_encoders.get(encoder), 1)
            else:
                print("Incorrect encoder.")
            
        def move_with_encoders(self, distance):
            print(f"Moving robot by {distance} centimetres")
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
                    time.sleep(0.001)
                    
                    # Calculate distance change and total distance travelled
                    distance_change, _ = self.calculate_current_pose()
                    distance_travelled += distance_change
                    print(f"Distance travelled: {distance_travelled} cm")
                    
                    # If the diatnce travelled is within the margin, stop the loop
                    if (distance_travelled >= distance - self.distance_margin) and (distance_travelled <= distance + self.distance_margin):
                        print(f"Robot succesfully moved to {self.current_pose}")
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
            
        def rotate_with_encoders(self, angle):
            print(f"Rotating robot by {angle} radians")
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
                    time.sleep(0.001)
                    
                    # Calculate distance change and total distance travelled
                    _, angle_change = self.calculate_current_pose()
                    angle_travelled += angle_change
                    print(f"Angle travelled: {angle_travelled} rad")
                    
                    # If the angle travelled is within the margin, stop the loop
                    if (angle_travelled >= angle - self.rotational_margin) and (angle_travelled <= angle + self.rotational_margin):
                        print(f"Robot succesfully rotated by {angle} radians")
                        print(f"New robot pose: {self.current_pose}")
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
            # Initialise the camera and create a reference to it
            self.camera = picamera.PiCamera()
            self.camera.rotation = 180
            self.camera.resolution = (640, 480) # HD: (1280, 960)
            self.camera.framerate = 32
            self.rawCapture = picamera.array.PiRGBArray(self.camera, size=self.camera.resolution)
            
            # Allow the camera time to warm up
            time.sleep(0.1)
            
            # Setup aruco dictionary and parameters
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            self.aruco_parameters = aruco.DetectorParameters_create()
            
            # Create counter for FPS
            self.frame_count = 0
            self.start_time = time.time()
        
        # Capture a single frame from the camera
        def capture_frame(self):
            frame = self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)
            image = frame.array
            
            # Show the frame
            cv2.imshow("Frame", image)
            
            # Clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            return image
        
        def capture_frame_continous(self):
            # Capture frames from the camera
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                image = frame.array
                
                # Show the frame
                cv2.imshow("Frame", image)

                # Clear the stream in preparation for the next frame
                self.rawCapture.truncate(0)

                # if the `q` key was pressed, break from the loop
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        
        def detect_aruco(self, image):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners,ids,rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
            
            frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
           
            self.frame_count += 1
            average_fps = self.frame_count / ( time.time() - self.start_time )
            cv2.putText(frame_markers,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

            # Show the frame
            cv2.imshow("Frame", frame_markers)
        
        def detect_blobs(self, image):
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            blueMin= (105,80,45)
            blueMax= (155,255,230)
            mask=cv2.inRange(hsv,blueMin,blueMax)
            params = cv2.SimpleBlobDetector_Params()
            params.thresholdStep = 255
            params.minRepeatability = 1
            params.blobColor = 255
            params.filterByInertia = False
            params.filterByConvexity = False

            #To just detect colour:
            #params.filterByArea = False

            #To detect colour and area:
            params.filterByArea = True
            params.minArea = 100
            params.maxArea = 80000 # 20000 if HD

            detector = cv2.SimpleBlobDetector_create(params)
            keypoints = detector.detect(mask)
            kp_image = cv2.drawKeypoints(mask,keypoints,None,color=(0,0,255),flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            self.frame_count += 1
            average_fps = self.frame_count / ( time.time() - self.start_time )
            cv2.putText(kp_image,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)

            #masked_image = cv2.bitwise_and(image,image,mask=mask)
            # Show the frame
            cv2.imshow("Frame", kp_image)
        
        def detect_color(self, image, color_value, image_format="hsv"):
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            blueMin= (105,80,45)
            blueMax= (155,255,230)
            mask=cv2.inRange(hsv,blueMin,blueMax)
            masked_image = cv2.bitwise_and(image,image,mask=mask)

            self.frame_count += 1
            average_fps = self.frame_count / ( time.time() - self.start_time )
            cv2.putText(masked_image,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)
            # Show the frame
            cv2.imshow("Frame", masked_image)
            
        def generate_aruco_tags(self, filename):
            filename = f"{filename}.pdf"
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

            fig = plt.figure()
            nx = 3
            ny = 2
            for i in range(1, nx*ny+1):
                ax = fig.add_subplot(ny,nx, i)
                img = aruco.drawMarker(aruco_dict,i, 700)
                plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
                ax.axis("off")

            plt.savefig(filename)
            plt.show()
            
    class Ultrasonic:
        def __init__(self): 
            self.REG_SEND_DATA_ULTRASOUND_1 = 20 # Ultrasound 1 send data 
            self.REG_SEND_DATA_ULTRASOUND_2 = 21 # Ultrasound 2 send data
        
        # Get data from the appropiate register based on target sensor
        def get_ultrasound_distance(self, sensor): 
            possible_sensors = {"left": self.REG_SEND_DATA_ULTRASOUND_1,
                                "right": self.REG_SEND_DATA_ULTRASOUND_2}
            
            if sensor in possible_sensors:
                distance = getRegister(possible_sensors.get(sensor))
                distance = np.uint8(distance)
                return distance
            else:
                print("Incorrect sensor name.")
                                
    class Joystick:
        def __init__(self):
            self.REG_SEND_MSG_JOYSTICK = 60 # Joystick send messages
            self.REG_RECEIVE_MSG_JOYSTICK = 61 # Joystic receive messages
            self.REG_SEND_DATA_JOYSTICK = 62 # Joystick send data (direction)
        
        # Get current joystick direction
        def get_joystick_direction(self):
            direction_data = getRegister(self.REG_SEND_DATA_JOYSTICK)
            
            possible_directions = {0: "stop", 1: "forward", 2:"backward", 
                                   3:"left", 4:"right", 5:"stop"}
            
            if direction_data in possible_directions:
                direction = possible_directions.get(direction_data)
                print(f"The joystick is moving {direction}")
                return direction
            else:
                print(f"Unusual value read from the register: {direction_data}")
            
    class InfraredSensor:
        def __init__(self):
            # I2C MUX communication 
            self.REG_SEND_IR = 10 # Serial register to send the IR data to
        
            # Get data from the register
        def get_infrared_distance(self):
            return getRegister(self.REG_SEND_IR)
