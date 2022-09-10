import cv2
from .ColorFilter import ColorFilter


def read_curr_code(bgr):

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    filter = ColorFilter(hsv, bgr)

    white = filter.white_filter()
    black = filter.black_filter()
    red_or_orange = filter.red_orange_filter(white) # works
    green = filter.green_filter()

    return black 