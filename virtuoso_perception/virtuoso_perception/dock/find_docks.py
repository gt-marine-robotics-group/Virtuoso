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

        self.node = None
    
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

        red_code = self.find_red_code(filter)
        node.get_logger().info(str(red_code))
    
    def find_axis_range(self, target, alpha):
        if target is None:
            return (-1, -1)
        dist = self.image_dimensions[0] * alpha
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
        
