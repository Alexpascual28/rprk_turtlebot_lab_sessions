from ARBPi import *
from TurtleBot import TurtleBot

import time
import math
import threading
import cv2
import numpy as np
import random

import getkey

import shape_detection

class KeyboardControl:
    def __init__(self):
        self.user_input = '';
        self.keyboard_thread = threading.Thread(target=self.main)

    def start_keyboard(self):
        self.keyboard_thread.start()

    def get_user_input(self):
        return self.user_input

    def main(self):
        while True:
            # Read user input
            self.user_input = getkey.getkey(blocking = True)

class ControlWASD:
    def __init__(self, turtlebot):
        self.tb = turtlebot
        self.keyboard = KeyboardControl()

        self.wasd_control_thread = threading.Thread(target=self.main)
        self.exit_flag = False

        self.keyboard.start_keyboard()

        self.previous_speed_input = ''
        self.previous_direction_input = ''

    def start_wasd_control(self):
        self.wasd_control_thread.start()

    def main(self):
        print("Enter robot direction (WASD) or speed (0-9):")

        while self.exit_flag == False:
            # Read user input
            user_input = self.keyboard.get_user_input()

            speedInput = ("0", "1", "2", "3", "4", "5", "6", "7", "8", "9")

            # If the user input is 0-9
            if user_input in speedInput:
                if user_input != self.previous_speed_input:
                    self.tb.motors.set_robot_speed_by_level(int(user_input))
                    self.previous_speed_input = user_input


            directionInput = {"W": "forward", "A": "left", "S": "backward", "D": "right",
                            "w": "forward", "a": "left", "s": "backward", "d": "right"}

            # If the user input is WASD
            if user_input in directionInput:
                if user_input != self.previous_direction_input:
                    direction = directionInput.get(user_input) # Get associated signal in directionInput
                    self.tb.motors.change_direction(direction)
                    self.previous_direction_input = user_input

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        tb = TurtleBot()
        wasd_control = ControlWASD(tb);

        tb.camera.start_image_acquisition(show_feed=False)
        wasd_control.start_wasd_control()

        while(True):
            source = tb.camera.get_current_image()

            if source is not None:
                image = source.copy()
                frame_aruco, corners, ids = tb.camera.detect_aruco(image, show_frame=False)

                blurred_image = shape_detection.blurring(image)

                colours = ["red", "blue", "green", "lilac"]

                for colour in colours:
                    mask, masked_image = tb.camera.detect_colour(blurred_image, colour, show_frame=False, frame_name=colour)

                    mask = shape_detection.dilating(mask)
                    mask = shape_detection.opening(mask)

                    mask = shape_detection.eroding(mask)
                    mask = shape_detection.closing(mask)
                    
                    mask = shape_detection.canny_edge_detection(mask)
                    mask = shape_detection.dilating(mask)

                    if mask is not None:
                        shapes = shape_detection.detect_shapes(mask)

                        for shape in shapes:
                            cv2.drawContours(image, shape.contour, 0, (0,255,0), 3)

                            if shape.shape_name != 'undefined':
                                cv2.putText(image, colour + " " + shape.shape_name, (shape.x,shape.y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)
                
                # Draw aruco locations
                result = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)

                # print("Showing Image")
                tb.camera.show_image("Current Image", result)

            else:
                # print("Not Showing Image")
                pass

    except KeyboardInterrupt:
        print('Interrupted!')