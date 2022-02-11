import cv2
import numpy as np
from .ColorFilter import ColorFilter

class DetectedBuoy():

    def __init__(self, color, x, y, w, h):
        self.color = color
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area = w * h


def find_buoys_for_color(bgr, color):

    canny = cv2.Canny(bgr, 50, 150)

    kernel = np.ones((3))

    dilated = cv2.dilate(canny, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    buoys = []

    for cnt in contours:

        area = cv2.contourArea(cnt)

        if area < 1000: continue 

        x, y, w, h = cv2.boundingRect(cnt)

        buoys.append(DetectedBuoy(color, x, y, w, h))
    
    return buoys

def find_buoys(bgr):

    hsv:np.ndarray = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    filter = ColorFilter(hsv, bgr)

    white = filter.white_filter()
    black = filter.black_filter()
    red_or_orange = filter.red_orange_filter(white)
    green = filter.green_filter()

    white_buoys = find_buoys_for_color(white, 'white')
    black_buoys = find_buoys_for_color(black, 'black')
    red_or_orange_buoys = find_buoys_for_color(red_or_orange, 'red_or_orange')
    green_buoys = find_buoys_for_color(green, 'green')

    buoys = white_buoys + black_buoys + red_or_orange_buoys + green_buoys

    return buoys
