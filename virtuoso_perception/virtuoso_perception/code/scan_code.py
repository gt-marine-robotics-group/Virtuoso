from collections import deque
import numpy as np
import cv2
from ..utils.ColorFilter import ColorFilter
from ..utils.code_identification import find_contours
from ..utils.math import distance

class ScanCode:

    def __init__(self, filter_bounds, code_loc_noise):

        self.image:np.ndarray = None

        self._filter_bounds = filter_bounds
        self._code_loc_noise = code_loc_noise

        self.code_found = None

        self._codes = deque(maxlen=2) # [['red', 'green', 'blue], ['red', 'green', 'blue']]
        self._curr_raw_data = deque(maxlen=5)
        self._curr_code_found = list()

        self._code_coords = dict()
        self._coord_sizes = dict()
        self._code_coord = None
        self._code_coord_iteration = 0
        self._coord_largest_size = None
    
    def read_code(self):

        if self.image is None:
            return
        
        coord = self._get_code_coord()

        if coord is None:
            return
        
        # self.get_logger().info(f'coord: {coord}')
        
        curr_code = self._read_curr_code(coord)

        self._curr_raw_data.append(curr_code)

        # self.get_logger().info(str(self.curr_raw_data))

        if len(self._curr_raw_data) < 5:
            return
        
        curr_code = self._find_mode()

        self._add_curr_code(curr_code) 

        # self.get_logger().info(str(self.curr_code_found))

        if len(self._curr_code_found) == 4:
            self._update_codes()
    
    def _update_codes(self):
        self._codes.append(self._curr_code_found)
        self._curr_code_found = list()
        if len(self._codes) == 2:
            self._check_for_finalized_code()

    def _check_for_finalized_code(self):
        if self._codes[0] != self._codes[1]:
            return
        self.code_found = self._codes[0][1:]
        # self.code_published = True
        # msg = Int32MultiArray()
        # msg.data = self.codes[0][1:]
        # self.code_pub.publish(msg)
        # self.finalized_code = self.codes[0][1:-1]
        # self.get_logger().info(f'FOUND CODE: {self.codes[0][1:]}')
    
    def _add_curr_code(self, code):
        # make sure the first code we add is black
        if len(self._curr_code_found) == 0 and code != -1:
            return
        
        if code == -1 and len(self._curr_code_found) > 0:
            return
        
        if len(self._curr_code_found) > 0 and code == self._curr_code_found[-1]:
            return
        
        self._curr_code_found.append(code)
    
    def _find_mode(self):
        counts = dict()
        curr_mode = None
        for d in self._curr_raw_data:
            if d in counts:
                counts[d] += 1
            else:
                counts[d] = 0
            if curr_mode is None or counts[d] > counts[curr_mode]:
                curr_mode = d
        return curr_mode

    def _get_code_coord(self):

        if not self._code_coord is None:
            return self._code_coord
        
        self._code_coord_iteration += 1

        coord, size = self._find_code_coords_and_size()

        if coord is None or size is None:
            return None

        # self.get_logger().info(f'coord: {coord}')
        for key, value in self._code_coords.items():
            if distance(key, coord) < self._code_loc_noise: # PARAM 10
                self._code_coords.pop(key)
                prevSize = self._coord_sizes.pop(key)
                newKey = self._calc_new_avg(key, value, coord)
                self._code_coords[newKey] = value + 1
                self._coord_sizes[newKey] = ((prevSize * value) + size) / (value + 1)

                if ((self._coord_largest_size is None or 
                    not self._coord_largest_size in self._coord_sizes or
                    self._coord_sizes[newKey] > self._coord_sizes[self._coord_largest_size]) 
                    and (value + 1 > 4)):
                    self._coord_largest_size = newKey
                # self.coord_sizes
                # if value + 1 > 9:
                #     self.code_coord = newKey
                #     return self.code_coord
                break
        else: # if we never break
            self._code_coords[coord] = 1
            self._coord_sizes[coord] = size
        
        if self._code_coord_iteration < 50:
            return None
        
        self.code_coord = self._coord_largest_size
        
        return self.code_coord

    def _find_code_coords_and_size(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        filter = ColorFilter(hsv, self.image)

        # No other red objects in background to confuse filter
        red_or_orange = filter.red_orange_filter()

        # image = CvBridge().cv2_to_imgmsg(red_or_orange, encoding='bgr8')
        # node.debug_pub.publish(image)
        # node.get_logger().info('finding coord!')

        return self._find_display_box(red_or_orange)

    def _find_display_box(self, bgr, targetCoord=None):

        contours = find_contours(bgr)

        closestCoord = None
        closestArea = None

        for cnt in contours:

            x, y, w, h = cv2.boundingRect(cnt)

            # if not node is None:
            #     node.get_logger().info(str((x, y)))
            #     node.get_logger().info(str(w * h))
            #     node.get_logger().info('----------')

            if targetCoord is None:
                if closestArea is None or closestArea < w * h:
                    closestCoord = (x, y)
                    closestArea = w * h
                continue

            if closestCoord is None or (distance((x, y), targetCoord) < 10 
                and (w * h) > closestArea):
                closestCoord = (x, y)
                closestArea = w * h

        return closestCoord, closestArea
    
    def _read_curr_code(self, coord):

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        filter = ColorFilter(hsv, self.image)

        red_or_orange = filter.red_orange_filter()
        green = filter.green_filter()
        blue = filter.blue_filter()

        boxes = [ # [r, g, b]
            self._find_display_box(red_or_orange, coord),
            self._find_display_box(green, coord),
            self._find_display_box(blue, coord)
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

    def _calc_new_avg(self, original, count, new):
        return (((original[0] * count) + new[0]) / (count + 1), ((original[1] * count) + new[1]) / (count + 1))