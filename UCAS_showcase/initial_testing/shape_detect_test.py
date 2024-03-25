from turtle import shape
from ARBPi import *
from TurtleBot import TurtleBot

import time
import math
import threading
import cv2
import numpy as np
import random

import shape_detection

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        tb = TurtleBot()

        tb.camera.start_image_acquisition(show_feed=False)

        while(True):
            print("Thread 1")
            source = tb.camera.get_current_image()

            if source is not None:
                image = source.copy()

                blurred_image = shape_detection.blurring(image)

                mask, masked_image = tb.camera.detect_colour(blurred_image, "red", show_frame=False, frame_name="red frame")

                # mask = shape_detection.dilating(mask)
                # mask = shape_detection.opening(mask)

                # mask = shape_detection.eroding(mask)
                # mask = shape_detection.closing(mask)
                
                # mask = shape_detection.canny_edge_detection(mask)
                # mask = shape_detection.dilating(mask)

                if mask is not None:
                    shapes = shape_detection.detect_shapes(mask)

                    for geometry in shapes:
                        cv2.drawContours(image, geometry.contour, 0, (0,255,0), 3)
                        cv2.putText(image, "red" + " " + geometry.shape_name, (geometry.x,geometry.y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA)
                
                # Draw aruco locations
                # result = cv2.aruco.drawDetectedMarkers(image, corners, ids)
                        
                  # red_mask, _ = tb.camera.detect_colour(image, "red", show_frame=False)
                # blue_mask, _ = tb.camera.detect_colour(image, "blue", show_frame=False)
                # green_mask, _ = tb.camera.detect_colour(image, "green", show_frame=False)
                # lilac_mask, _ = tb.camera.detect_colour(image, "lilac", show_frame=False)

                # add masks
                # colour_mask = red_mask | blue_mask | green_mask | lilac_mask

                # Mask image with colour masks
                # masked_image = cv2.bitwise_and(image,image,mask=colour_mask)

                print("Showing Image")
                tb.camera.show_image("Current Image", image)
                tb.camera.show_image("Current Mask", mask)

            else:
                print("Not Showing Image")

    except KeyboardInterrupt:
        print('Interrupted!')