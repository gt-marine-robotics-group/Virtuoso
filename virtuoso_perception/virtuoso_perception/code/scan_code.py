from math import sqrt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Int8
from cv_bridge import CvBridge
import cv2
from ..utils.scan_code import read_curr_code, find_code_coords_and_size
import numpy as np
from collections import deque

class ScanCode(Node):

    def __init__(self):
        super().__init__('scan_code')

        # self.camera_sub = self.create_subscription(Image, '/downscaled_image', self.image_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.image_callback, 10)
        self.get_code_sub = self.create_subscription(Int8, '/perception/get_code', 
            self.start_scan, 10)
        self.code_pub = self.create_publisher(Int32MultiArray, '/perception/code', 10)

        self.debug_pub = self.create_publisher(Image, '/perception/debug', 10)

        self.image = None
        # self.scan_requested = False
        self.scan_requested = True

        # scan twice (or more) to verify code is correct before publishing
        self.codes = deque(maxlen=2) # [['red', 'green', 'blue], ['red', 'green', 'blue']]
        self.curr_raw_data = deque(maxlen=5)
        self.curr_code_found = list()

        self.code_published = False

        self.code_coords = dict()
        self.code_coord = None

        self.create_timer(.1, self.read_code)
    
    def start_scan(self, msg):
        self.get_logger().info('Received Scan Code Request')
        self.scan_requested = True
    
    def find_mode(self, data):
        counts = dict()
        curr_mode = None
        for d in data:
            if d in counts:
                counts[d] += 1
            else:
                counts[d] = 0
            if curr_mode is None or counts[d] > counts[curr_mode]:
                curr_mode = d
        return curr_mode
    
    def add_curr_code(self, code):
        # make sure the first code we add is black
        if len(self.curr_code_found) == 0 and code != -1:
            return

        if code in self.curr_code_found:
            return
        self.curr_code_found.append(code)
    
    def update_codes(self):
        self.codes.append(self.curr_code_found)
        self.curr_code_found = list()
        if len(self.codes) == 2:
            self.check_for_finalized_code()
    
    def check_for_finalized_code(self):
        if self.codes[0] != self.codes[1]:
            return
        self.code_published = True
        msg = Int32MultiArray()
        msg.data = self.codes[0][1:]
        self.code_pub.publish(msg)
        # self.finalized_code = self.codes[0][1:-1]
        self.get_logger().info(f'FOUND CODE: {self.codes[0][1:]}')

    def image_callback(self, msg:Image):
        self.image = msg
    
    def get_code_coord(self, bgr):
        if not self.code_coord is None:
            return self.code_coord

        coord, size = find_code_coords_and_size(bgr, self)

        if coord is None or size is None:
            return None

        if size < 500:
            return

        self.get_logger().info(f'coord: {coord}')
        for key, value in self.code_coords.items():
            if self.distance(key, coord) < 10:
                self.code_coords.pop(key)
                newKey = self.calc_new_avg(key, value, coord)
                self.code_coords[newKey] = value + 1
                if value + 1 > 9:
                    self.code_coord = newKey
                    return self.code_coord
                break
        else: # if we never break
            self.code_coords[coord] = 1
        
        return self.code_coord

    def read_code(self):
        # self.get_logger().info(str(self.scan_requested))

        if not self.scan_requested:
            return

        if self.code_published:
            return

        if self.image is None:
            return

        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        coord = self.get_code_coord(bgr)

        if coord is None:
            return
        
        curr_code = read_curr_code(bgr, coord)

        self.curr_raw_data.append(curr_code)

        self.get_logger().info(str(self.curr_raw_data))

        if len(self.curr_raw_data) < 5:
            return
        
        curr_code = self.find_mode(self.curr_raw_data)

        self.add_curr_code(curr_code) 

        self.get_logger().info(str(self.curr_code_found))

        if len(self.curr_code_found) == 4:
            self.update_codes()

    def distance(self, a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def calc_new_avg(self, original, count, new):
        return (((original[0] * count) + new[0]) / (count + 1), ((original[1] * count) + new[1]) / (count + 1))

def main(args=None):
    rclpy.init(args=args)
    node = ScanCode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()