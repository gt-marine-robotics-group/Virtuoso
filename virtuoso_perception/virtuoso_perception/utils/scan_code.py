import math
import cv2
from cv_bridge import CvBridge
from .ColorFilter import ColorFilter
import numpy as np

def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def find_display_box(bgr, targetCoord=None, node=None):
    canny = cv2.Canny(bgr, 50, 150)

    kernel = np.ones((3))

    dilated = cv2.dilate(canny, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    closestCoord = None
    closestArea = None

    for cnt in contours:

        x, y, w, h = cv2.boundingRect(cnt)

        if not node is None:
            node.get_logger().info(str((x, y)))
            node.get_logger().info(str(w * h))
            node.get_logger().info('----------')

        if targetCoord is None:
            if closestArea is None or closestArea < w * h:
                closestCoord = (x, y)
                closestArea = w * h
            continue
        # else:
        #     if closestArea is None:
        #         closestArea = w * h


        if closestCoord is None or (distance((x, y), targetCoord) < 10 
            and (w * h) > closestArea):
            closestCoord = (x, y)
            closestArea = w * h

    return closestCoord, closestArea

def find_code_coords_and_size(bgr, node):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    filter = ColorFilter(hsv, bgr)

    white = filter.white_filter()
    # No other red objects in background to confuse filter
    red_or_orange = filter.red_orange_filter(white)
    # red_or_orange = filter.blue_filter()

    image = CvBridge().cv2_to_imgmsg(red_or_orange, encoding='bgr8')
    node.debug_pub.publish(image)

    return find_display_box(red_or_orange)

def read_curr_code(bgr, coord):

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    filter = ColorFilter(hsv, bgr)

    white = filter.white_filter()
    black = filter.black_filter()
    red_or_orange = filter.red_orange_filter(white)
    green = filter.green_filter()
    blue = filter.blue_filter()

    boxes = [ # [r, g, b]
        find_display_box(red_or_orange, coord),
        find_display_box(green, coord),
        find_display_box(blue, coord)
    ]

    curr_code = -1

    for i, box in enumerate(boxes):
        if box[0] is None or box[1] is None:
            continue
        if box[1] < 500 or distance(box[0], coord) > 30:
            continue
        if curr_code == -1 or distance(box[0], coord) < distance(boxes[curr_code][0], coord):
            curr_code = i

    return curr_code


