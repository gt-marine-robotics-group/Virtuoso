from collections import deque
from rclpy.node import Node
from ..utils.code_identification import find_contours
from ..utils.ColorFilter import ColorFilter
import cv2
import numpy as np

class FindDockCodes:

    code_loc_weights = {4: .4, 3: .3, 2: .15, 1: .1, 0: .05}
    code_colors = ['red', 'green', 'blue']

    def __init__(self, filter_bounds):

        self.image:np.ndarray = None
        self._image_dimensions = (0, 0) # (height, width)

        self._filter_bounds = filter_bounds

        self._code_locations = {
            'red': deque(maxlen=5),
            'blue': deque(maxlen=5),
            'green': deque(maxlen=5)
        }
        self._code_locations_count = 0

        self._code_x_offsets = {
            'red': None,
            'blue': None,
            'green': None
        }

        self._prev_code_x_offsets = deque(maxlen=10)

        self.node = None # for debugging
    
    def _debug(self, msg):
        if self.node is None: return
        self.node.get_logger().info(msg)
    
    def find_docks(self, node:Node=None):
        
        if self.image is None:
            return None
        
        self.node = node
        
        # bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        self._image_dimensions = self.image.shape[:2]

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        filter = ColorFilter(hsv, self.image)

        self._update_code_locations(filter)

        if (len(self._code_locations['red']) < 5):
            return None
        
        self._find_code_x_offsets()
        # self.node.get_logger().info(f'Map Offsets: {self.code_x_offsets}')
        self._debug(f'Map Offsets: {self._code_x_offsets}')

        self._prev_code_x_offsets.append(self._code_x_offsets)

        if len(self._prev_code_x_offsets) < 10:
            return
        
        if not self._prev_offsets_consistent():
            self._debug('Not consistent!')
            return

        return self._format_x_offsets()
    
    def _prev_offsets_consistent(self):
        offset_bounds = {
            'red': [None, None],
            'green': [None, None],
            'blue': [None, None]
        }

        for offsets in self._prev_code_x_offsets:
            for color in FindDockCodes.code_colors:
                offset = offsets[color]
                if offset is None:
                    return False
                if offset_bounds[color][0] is None or offset < offset_bounds[color][0]:
                    offset_bounds[color][0] = offset
                if offset_bounds[color][1] is None or offset > offset_bounds[color][1]:
                    offset_bounds[color][1] = offset
        
        offset_range = self._image_dimensions[1] * 0.05
        for color in FindDockCodes.code_colors:
            if offset_bounds[color][1] - offset_bounds[color][0] > offset_range:
                return False
        
        return True
    
    def _format_x_offsets(self):
        arr = list(0 for _ in range(7))
        for i, color in enumerate(FindDockCodes.code_colors):
            offset = self._code_x_offsets[color]
            if offset is None:
                arr[i + 3] = 1
            else:
                arr[i] = offset
        arr[6] = self._image_dimensions[1]
        return arr
    
    def _find_weighted_x_avg(self, color):
        if len(self._code_locations[color]) < 5:
            return None
        total = sum(
            FindDockCodes.code_loc_weights[i] * (code[0][0] + (code[1][0]/2))
            for i, code in enumerate(self._code_locations[color])
        )
        return total
        
    def _find_code_x_offsets(self):
        mid_x = self._image_dimensions[1] / 2

        for color in FindDockCodes.code_colors:
            avg = self._find_weighted_x_avg(color)
            if avg is None:
                self._code_x_offsets[color] = None
            else:
                self._code_x_offsets[color] = -1 * (avg - mid_x)
    
    def _find_axis_range(self, target, alpha):
        if target is None:
            return (-1, -1)
        dist = self._image_dimensions[0] * alpha # cv2 gives coordinates in y,x
        lower = target - dist
        upper = target + dist
        return (lower, upper)
    
    def _find_largest_rect_on_axis(self, bgr, target=None, target_area=None):
        contours = find_contours(bgr)

        coord = None
        coord_dimensions = (0, 0)

        axis_range = self._find_axis_range(target, .05) # PARAM

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
            
            if target_area is None and w * h > self._area(coord_dimensions):
                coord = (x, y)
                coord_dimensions = (w, h)
                continue

            if (target_area is not None and 
                abs(w * h - target_area) < abs(self._area(coord_dimensions) - target_area)):
                coord = (x, y)
                coord_dimensions = (w, h)
        
        return coord, coord_dimensions
    
    def _find_red_code(self, filter:ColorFilter):
        red_or_orange = filter.red_orange_filter(hsv_lower1=self._filter_bounds['red']['lower1'],
            hsv_upper1=self._filter_bounds['red']['upper1'], 
            hsv_lower2=self._filter_bounds['red']['lower2'],
            hsv_upper2=self._filter_bounds['red']['upper2'])

        return self._find_largest_rect_on_axis(red_or_orange)
    
    def _find_blue_code(self, filter:ColorFilter, target, target_area):
        blue = filter.blue_filter(hsv_lower=self._filter_bounds['blue']['lower'],
            hsv_upper=self._filter_bounds['blue']['upper'])
        return self._find_largest_rect_on_axis(blue, target, target_area)
    
    def _find_green_code(self, filter:ColorFilter, target, target_area):
        green = filter.green_filter(hsv_lower=self._filter_bounds['green']['lower'],
            hsv_upper=self._filter_bounds['green']['upper'])
        return self._find_largest_rect_on_axis(green, target, target_area)

    def _area(self, dim):
        return dim[0] * dim[1]
    
    def _update_code_locations(self, filter:ColorFilter):
        red_coord, red_dimensions = self._find_red_code(filter)
        # self.node.get_logger().info(f'Red coord: {red_coord}, area: {self.area(red_dimensions)}')
        self._debug(f'Red coord: {red_coord}, area: {self._area(red_dimensions)}')

        if red_coord is None: return

        axis = red_coord[1] + (.5 * red_dimensions[1])
        blue_coord, blue_dimensions = self._find_blue_code(filter, axis, 
            self._area(red_dimensions)) # red_code is (x,y) not (y,x)
        green_coord, green_dimensions = self._find_green_code(filter, axis,
            self._area(red_dimensions))

        # self.node.get_logger().info(f'Blue coord: {blue_coord}, area: {self.area(blue_dimensions)}')
        # self.node.get_logger().info(f'Green coord: {green_coord}, area: {self.area(green_dimensions)}')
        self._debug(f'Blue coord: {blue_coord}, area: {self._area(blue_dimensions)}')
        self._debug(f'Green coord: {green_coord}, area: {self._area(green_dimensions)}')

        self._code_locations['red'].append((red_coord, red_dimensions, self._code_locations_count))
        if blue_coord is not None:
            self._code_locations['blue'].append((blue_coord, blue_dimensions, 
                self._code_locations_count))
        if green_coord is not None:
            self._code_locations['green'].append((green_coord, green_dimensions, 
                self._code_locations_count))
        self._code_locations_count += 1


        
        
