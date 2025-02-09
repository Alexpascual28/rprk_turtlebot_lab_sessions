from ARBPi import *
from TurtleBot import TurtleBot

import time
import math
import threading
import cv2
import numpy as np
import random

class WallFollowerFSM:
    def __init__(self, turtlebot):
        self.is_initialised = False
        self.lower_left_wall_threshold = 5
        self.middle_left_wall_threshold = 7
        self.upper_left_wall_threshold = 10
        self.right_wall_threshold = 7
        self.front_wall_threshold = 12

        self.tb = turtlebot

        self.state_machine = {
                    0: ["Start", self.initialise, (lambda: self.is_initialised,), (1, 0)],
                    1: ["Move Forward", self.move_forward, (self.obstacle_in_front, self.wall_to_the_right, lambda: not self.wall_to_the_left(), self.left_wall_close, self.left_wall_far), (4, 3, 3, 2, 3, 1)],
                    2: ["Adjust Right", self.adjust_right, (self.obstacle_in_front, self.wall_to_the_right, lambda: not self.wall_to_the_left(), self.left_wall_close, self.left_wall_far), (4, 3, 1, 2, 3, 1)],
                    3: ["Adjust Left", self.adjust_left, (self.obstacle_in_front, self.wall_to_the_right, lambda: not self.wall_to_the_left(), self.left_wall_close, self.left_wall_far), (4, 3, 1, 2, 3, 1)],
                    4: ["Adjust Backwards", self.adjust_backwards, (self.obstacle_in_front, self.wall_to_the_right, lambda: not self.wall_to_the_left(), self.left_wall_close, self.left_wall_far), (4, 3, 1, 2, 3, 1)],
                    }

        self.state_index = 0
        self.exit_flag = False
        self.is_initialised =  True

        self.wall_follower_thread = threading.Thread(target=self.main)

    def start_wall_follower(self):
        self.wall_follower_thread.start()

    def main(self):
        while self.exit_flag == False:
            current_state = self.state_machine[self.state_index]

            name = current_state[0];
            print("Current state:", name);

            action = current_state[1]
            
            action()

            for index, condition in enumerate(current_state[2] + (lambda: True,)):
                if condition() == True:
                    self.state_index = current_state[3][index]
                    break;

            time.sleep(0.2)

    def initialise(self):
        print("Initialising robot...")
        print("Initialised") if self.is_initialised == True else print("Not initialised")

    def move_forward(self):
        self.tb.motors.change_direction("forward");
        self.tb.motors.set_robot_speed_by_level(7);

    def adjust_backwards(self):
        self.tb.motors.change_direction("backward");
        self.tb.motors.set_robot_speed_by_level(5);

        time.sleep(0.2)

        self.tb.motors.change_direction("right");

        time.sleep(0.7)

    def adjust_left(self):
        self.tb.motors.change_direction("left");
        self.tb.motors.set_robot_speed_by_level(3);

        time.sleep(0.7)

        self.tb.motors.change_direction("forward");
        self.tb.motors.set_robot_speed_by_level(7);


    def adjust_right(self):
        self.tb.motors.change_direction("right");
        self.tb.motors.set_robot_speed_by_level(3);

        time.sleep(0.7)

        self.tb.motors.change_direction("forward");
        self.tb.motors.set_robot_speed_by_level(7);

    def obstacle_in_front(self):
        ir_distance = self.tb.infrared.get_infrared_distance()
        return ir_distance < self.front_wall_threshold

    def wall_to_the_right(self):
        right_distance = self.tb.ultrasound.get_ultrasound_distance("right")
        return right_distance < self.right_wall_threshold

    def wall_to_the_left(self):
        left_distance = self.tb.ultrasound.get_ultrasound_distance("left")
        return left_distance < self.upper_left_wall_threshold

    def left_wall_close(self):
        left_distance = self.tb.ultrasound.get_ultrasound_distance("left")
        return left_distance < self.lower_left_wall_threshold

    def left_wall_far(self):
        left_distance = self.tb.ultrasound.get_ultrasound_distance("left")
        return left_distance < self.upper_left_wall_threshold & left_distance > self.middle_left_wall_threshold

    def finalize(self):
        self.exit_flag = True
        print("Map explored. Finalizing program...")

class RobotVision:
    def __init__(self, turtlebot):
        self.tb = turtlebot
        self.tb.camera.start_image_acquisition(show_feed=False)

        self.robot_vision_thread = threading.Thread(target=self.main)

    def start_robot_vision(self):
        self.robot_vision_thread.start()

    def main(self):
        while(True):
            # print("Thread 1")
            source = self.tb.camera.get_current_image()

            if source is not None:
                image = source.copy()
                frame_aruco, corners, ids = self.tb.camera.detect_aruco(image, show_frame=False)

                for colour in ["red", "blue", "green", "lilac"]:
                    mask, masked_image = self.tb.camera.detect_colour(image, colour, show_frame=False, frame_name=colour)

                    if mask is not None:
                        for shape_name in ["triangle", "rectangle", "star", "circle"]:
                            shapes = self.tb.camera.detect_shapes(mask, shape_name)

                            for shape in shapes:
                                cv2.putText(image, f"{colour} {shape[0]}", (shape[2],shape[3]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)
                                cv2.drawContours(image, shape[1], 0, (0,255,0), 3)
                                
                # Draw aruco locations
                result = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)

                # print("Showing Image")
                self.tb.camera.show_image("Current Image", result)

            else:
                # print("Not Showing Image")
                pass

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        tb = TurtleBot()
        wall_follower = WallFollowerFSM(tb);
        robot_vision = RobotVision(tb);

        wall_follower.start_wall_follower()
        robot_vision.start_robot_vision()

    except KeyboardInterrupt:
        print('Interrupted!')