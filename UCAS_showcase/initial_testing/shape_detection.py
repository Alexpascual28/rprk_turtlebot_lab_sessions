import cv2
import numpy as np

class Shape():
    def __init__(self, shape_name, contour, x, y, approx):
        self.shape_name = shape_name
        self.contour = contour
        self.x = x
        self.y = y
        self.approx = approx

def closing(mask):
    kernel = np.ones((7,7),np.uint8) 
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return closing

def opening(mask):
    kernel = np.ones((6,6),np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return opening

def blurring(mask):
    blur = cv2.GaussianBlur(mask,(5,5),0)
    return blur

def eroding(mask):
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 1)
    return erosion

def dilating(mask):
    kernel = np.ones((5,5),np.uint8)
    dilation = cv2.dilate(mask, kernel, iterations = 1)
    return dilation

def canny_edge_detection(mask):
    edges = cv2.Canny(mask,100,200)
    return edges

def detect_shapes(mask):
    shapes = []

    mask = dilating(mask)
    mask = opening(mask)

    mask = eroding(mask)
    mask = closing(mask)
    
    mask = canny_edge_detection(mask)
    mask = dilating(mask)

    contours, h = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key = len)

    for contour in contours[-3:]:
        #Amount of edges
        approx = cv2.approxPolyDP(contour, 0.02*cv2.arcLength(contour, True), True)

        #Center locations
        M = cv2.moments(contour)
        if M['m00'] == 0.0:
            continue
        centroid_x = int(M['m10']/M['m00'])
        centroid_y = int(M['m01']/M['m00'])

        if len(approx) == 3:
            shape_name = 'triangle'
        elif len(approx) == 4:
            shape_name = 'rectangle'
        elif len(approx) == 10:
            shape_name = 'star'
        elif len(approx) >= 11:
            shape_name = 'circle'
        else:
            shape_name ='undefined'

        shape = Shape(shape_name, contour, centroid_x, centroid_y, len(approx))
        shapes.append(shape)

    return shapes