from collections import deque
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from rclpy.node import Node
from ..utils.code_identification import find_contours
from ..utils.ColorFilter import ColorFilter
import cv2

class FindDocks:

    def __init__(self):

        self.image:Image = None
        self.image_dimensions = (0, 0) # (height, width)
        # self.search_requested:bool = False
        self.search_requested:bool = True

        self.code_locations = {
            'red': deque(maxlen=5),
            'blue': deque(maxlen=5),
            'green': deque(maxlen=5)
        }

        self.node = None # for debugging
    
    def get_ready_msg(self):
        if self.search_requested:
            return None
        msg = Int32() 
        msg.data = 1
        return msg
    
    def find(self, node:Node=None):

        if not self.search_requested:
            return
        
        if self.image is None:
            return
        
        self.node = node
        
        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        self.image_dimensions = bgr.shape[:2]

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        filter = ColorFilter(hsv, bgr)

        self.update_code_locations(filter)
    
    def find_axis_range(self, target, alpha):
        if target is None:
            return (-1, -1)
        dist = self.image_dimensions[0] * alpha # cv2 gives coordinates in y,x
        lower = target - dist
        upper = target + dist
        return (lower, upper)
    
    def find_largest_rect_on_axis(self, bgr, target=None):
        contours = find_contours(bgr)

        coord = None
        coord_area = None

        axis_range = self.find_axis_range(target, .05)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            mid_y = y + (h/2)

            if ((not target is None) 
                and mid_y > axis_range[1] or mid_y < axis_range[0]):
                continue

            if coord is None or w * h > coord_area:
                coord = (x, y)
                coord_area = w * h
        
        return coord, coord_area
    
    def find_red_code(self, filter:ColorFilter):
        white = filter.white_filter()
        red_or_orange = filter.red_orange_filter(white)

        return self.find_largest_rect_on_axis(red_or_orange)
    
    def find_blue_code(self, filter:ColorFilter, target):
        blue = filter.blue_filter()
        return self.find_largest_rect_on_axis(blue, target)
    
    def find_green_code(self, filter:ColorFilter, target):
        green = filter.green_filter()
        return self.find_largest_rect_on_axis(green, target)
    
    def update_code_locations(self, filter:ColorFilter):
        red_code, red_area = self.find_red_code(filter)
        self.node.get_logger().info(str(red_code))

        if red_code is None: return

        # blue_code, blue_area = self.find_blue_code(filter, red_code[1]) # red_code is (x,y) not (y,x)

        
        
