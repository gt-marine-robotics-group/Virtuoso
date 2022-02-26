import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
import numpy as np
from ...utils.ColorFilter import ColorFilter
from ...utils.identify_buoys import find_largest_buoy

class Red(Node):

    def __init__(self):
        super().__init__('largest_red_buoy')

        self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', self.image_callback, 10)

        self.timer = self.create_timer(.1, self.find_largest_red)

        # self.test_pub = self.create_publisher(Image, '/test_image', 10)

        self.image = None
    
    def image_callback(self, msg:Image):
        self.image = msg
    
    def find_largest_red(self):

        if (self.image is None): return

        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        hsv:np.ndarray = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        filter = ColorFilter(hsv, bgr)

        white = filter.white_filter()
        red_orange = filter.red_orange_filter(white)

        buoy = find_largest_buoy(red_orange, 'red')

        # self.test_pub.publish(CvBridge().cv2_to_imgmsg(red_orange, encoding='bgr8'))

        # self.get_logger().info(str(buoy))


def main(args=None):
    
    rclpy.init(args=args)

    node = Red()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()