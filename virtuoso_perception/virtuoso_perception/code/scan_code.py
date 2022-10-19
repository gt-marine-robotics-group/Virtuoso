from math import sqrt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ..utils.scan_code import read_curr_code, find_code_coords_and_size
import numpy as np

class ScanCode(Node):

    def __init__(self):
        super().__init__('scan_code')

        # self.camera_sub = self.create_subscription(Image, '/downscaled_image', self.image_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.image_callback, 10)
        self.debug_pub = self.create_publisher(Image, '/scan_code/debug', 10)

        self.image = None

        # scan twice (or more) to verify code is correct before publishing
        self.codes = [] # [['red', 'green', 'blue], ['red', 'green', 'blue']]
        self.is_black = False # code resets when it goes black

        self.code_coords = dict()
        self.code_coord = None

        self.create_timer(.1, self.read_code)

    def image_callback(self, msg:Image):
        self.image = msg
    
    def get_code_coord(self, bgr):
        if not self.code_coord is None:
            return self.code_coord

        coord, size = find_code_coords_and_size(bgr)

        self.get_logger().info(str((coord, size)))
        self.get_logger().info(str(self.code_coords))
        self.get_logger().info('--------')

        if coord is None or size is None:
            return None

        if size < 500:
            return

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

        if self.image is None:
            return

        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        # curr_code = read_curr_code(bgr)
        coord = self.get_code_coord(bgr)

        if coord is None:
            return
        
        self.get_logger().info(str(coord))

        curr_code = read_curr_code(bgr, coord)

        self.get_logger().info(str(curr_code))

        # self.get_logger().info(str(box))

        # image_msg = CvBridge().cv2_to_imgmsg(curr_code)
        # self.get_logger().info(str(np.shape(curr_code)))
        # self.get_logger().info(str(len(curr_code)))
        self.get_logger().info('-------')

        # self.debug_pub.publish(image_msg)

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