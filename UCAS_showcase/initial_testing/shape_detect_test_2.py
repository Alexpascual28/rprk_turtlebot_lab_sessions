import cv2
import numpy as np

from TurtleBot import TurtleBot
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

                if mask is not None:
                    cv2.imshow('1. original image', mask)
                    cv2.waitKey(0)

                    gray_img = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
                    cv2.imshow('2. gray image', gray_img)
                    cv2.waitKey(0)

                    edges = cv2.Canny(gray_img, 50, 200)
                    cv2.imshow('3. edges', edges)
                    cv2.waitKey(0)

                    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

                    for cnt in contours:

                        approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
                        
                        print(len(approx))
                        # detect shapes using cv2

                        if len(approx) == 3:
                            shape = 'Triangle'
                            M = cv2.moments(approx)
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])

                        elif len(approx) == 4:
                            x, y, w, h = cv2.boundingRect(cnt)
                            if abs(w-h) < 5:
                                shape = 'Square'
                                M = cv2.moments(approx)
                                cx = int(M['m10'] / M['m00'])
                                cy = int(M['m01'] / M['m00'])
                            else:
                                shape = 'Rectangle'
                                M = cv2.moments(approx)
                                cx = int(M['m10'] / M['m00'])
                                cy = int(M['m01'] / M['m00'])

                        elif len(approx) == 10:
                            shape = 'Star'
                            M = cv2.moments(approx)
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])

                        elif len(approx) == 8:
                            shape = 'Circle'
                            M = cv2.moments(approx)
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])

                        cv2.putText(image, shape, (cx-30, cy),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)

                        cv2.drawContours(image, cnt, -1, (0, 255, 0), 2)
                        
                    cv2.imshow('cnt', image)
                    cv2.destroyAllWindows()

            else:
                print("Not Showing Image")

    except KeyboardInterrupt:
        print('Interrupted!')