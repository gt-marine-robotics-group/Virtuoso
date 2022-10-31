from collections import deque
from rclpy.node import Node
from ..utils.code_identification import find_contours
from ..utils.ColorFilter import ColorFilter
import cv2
import numpy as np

class FindDocks:

    code_loc_weights = {4: .4, 3: .3, 2: .15, 1: .1, 0: .05}
    code_colors = ['red', 'green', 'blue']

    def __init__(self):

        self.image:np.ndarray = None
        self.image_dimensions = (0, 0) # (height, width)

        self.code_locations = {
            'red': deque(maxlen=5),
            'blue': deque(maxlen=5),
            'green': deque(maxlen=5)
        }
        self.code_locations_count = 0

        self.code_x_offsets = {
            'red': None,
            'blue': None,
            'green': None
        }

        self.node = None # for debugging
    
    def debug(self, msg):
        if self.node is None: return
        self.node.get_logger().info(msg)
    
    def find_docks(self, node:Node=None):
        
        if self.image is None:
            return None
        
        self.node = node
        
        # bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        self.image_dimensions = self.image.shape[:2]

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        filter = ColorFilter(hsv, self.image)

        self.update_code_locations(filter)

        if (len(self.code_locations['red']) < 5):
            return None
        
        self.find_code_x_offsets()
        # self.node.get_logger().info(f'Map Offsets: {self.code_x_offsets}')
        self.debug(f'Map Offsets: {self.code_x_offsets}')

        return self.format_x_offsets()
    
    def format_x_offsets(self):
        arr = list(0 for _ in range(7))
        for i, color in enumerate(FindDocks.code_colors):
            offset = self.code_x_offsets[color]
            if offset is None:
                arr[i + 3] = 1
            else:
                arr[i] = offset
        arr[6] = self.image_dimensions[1]
        return arr
    
    def find_weighted_x_avg(self, color):
        if len(self.code_locations[color]) < 5:
            return None
        total = sum(
            FindDocks.code_loc_weights[i] * (code[0][0] + (code[1][0]/2))
            for i, code in enumerate(self.code_locations[color])
        )
        return total
        
    def find_code_x_offsets(self):
        mid_x = self.image_dimensions[1] / 2

        for color in FindDocks.code_colors:
            avg = self.find_weighted_x_avg(color)
            if avg is None:
                self.code_x_offsets[color] = None
            else:
                self.code_x_offsets[color] = -1 * (avg - mid_x)
    
    def find_axis_range(self, target, alpha):
        if target is None:
            return (-1, -1)
        dist = self.image_dimensions[0] * alpha # cv2 gives coordinates in y,x
        lower = target - dist
        upper = target + dist
        return (lower, upper)
    
    def find_largest_rect_on_axis(self, bgr, target=None, target_area=None):
        contours = find_contours(bgr)

        coord = None
        coord_dimensions = (0, 0)

        axis_range = self.find_axis_range(target, .05)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            mid_y = y + (h/2)

            if ((not target is None) 
                and mid_y > axis_range[1] or mid_y < axis_range[0]):
                continue

            if coord is None:
                coord = (x, y)
                coord_dimensions = (w, h)
                continue
            
            if target_area is None and w * h > self.area(coord_dimensions):
                coord = (x, y)
                coord_dimensions = (w, h)
                continue

            if (target_area is not None and 
                abs(w * h - target_area) < abs(self.area(coord_dimensions) - target_area)):
                coord = (x, y)
                coord_dimensions = (w, h)
        
        return coord, coord_dimensions
    
    def find_red_code(self, filter:ColorFilter):
        white = filter.white_filter()
        red_or_orange = filter.red_orange_filter(white)

        return self.find_largest_rect_on_axis(red_or_orange)
    
    def find_blue_code(self, filter:ColorFilter, target, target_area):
        blue = filter.blue_filter()
        return self.find_largest_rect_on_axis(blue, target, target_area)
    
    def find_green_code(self, filter:ColorFilter, target, target_area):
        green = filter.green_filter()
        return self.find_largest_rect_on_axis(green, target, target_area)

    def area(self, dim):
        return dim[0] * dim[1]
    
    def update_code_locations(self, filter:ColorFilter):
        red_coord, red_dimensions = self.find_red_code(filter)
        # self.node.get_logger().info(f'Red coord: {red_coord}, area: {self.area(red_dimensions)}')
        self.debug(f'Red coord: {red_coord}, area: {self.area(red_dimensions)}')

        if red_coord is None: return

        axis = red_coord[1] + (.5 * red_dimensions[1])
        blue_coord, blue_dimensions = self.find_blue_code(filter, axis, 
            self.area(red_dimensions)) # red_code is (x,y) not (y,x)
        green_coord, green_dimensions = self.find_green_code(filter, axis,
            self.area(red_dimensions))

        # self.node.get_logger().info(f'Blue coord: {blue_coord}, area: {self.area(blue_dimensions)}')
        # self.node.get_logger().info(f'Green coord: {green_coord}, area: {self.area(green_dimensions)}')
        self.debug(f'Blue coord: {blue_coord}, area: {self.area(blue_dimensions)}')
        self.debug(f'Green coord: {green_coord}, area: {self.area(green_dimensions)}')

        self.code_locations['red'].append((red_coord, red_dimensions, self.code_locations_count))
        if blue_coord is not None:
            self.code_locations['blue'].append((blue_coord, blue_dimensions, 
                self.code_locations_count))
        if green_coord is not None:
            self.code_locations['green'].append((green_coord, green_dimensions, 
                self.code_locations_count))
        self.code_locations_count += 1


        
        
