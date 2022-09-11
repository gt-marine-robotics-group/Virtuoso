import math
import cv2
from .ColorFilter import ColorFilter
import numpy as np

def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def find_display_box(bgr, targetCoord=None, targetArea=None):
    canny = cv2.Canny(bgr, 50, 150)

    kernel = np.ones((3))

    dilated = cv2.dilate(canny, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    closestCoord = None
    closestArea = None

    for cnt in contours:

        x, y, w, h = cv2.boundingRect(cnt)

        if targetCoord is None:
            if closestArea is None or closestArea < w * h:
                closestCoord = (x, y)
                closestArea = w * h
            continue

        if closestCoord is None or (distance((x, y), targetCoord) < distance(closestCoord, targetCoord) 
        and (w * h) - targetArea < closestArea - targetArea):
            closestCoord = (x, y)
            closestArea = w * h

    return closestCoord, closestArea

def find_code_coords_and_size(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    filter = ColorFilter(hsv, bgr)

    white = filter.white_filter()
    # No other red objects in background to confuse filter
    red_or_orange = filter.red_orange_filter(white)

    return find_display_box(red_or_orange)

def read_curr_code(bgr, coord, area):

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    filter = ColorFilter(hsv, bgr)

    white = filter.white_filter()
    black = filter.black_filter()
    red_or_orange = filter.red_orange_filter(white)
    green = filter.green_filter()
    blue = filter.blue_filter()
