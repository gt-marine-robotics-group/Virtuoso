import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ..utils.scan_code import read_curr_code

class ScanCode(Node):

    def __init__(self):
        super().__init__('scan_code')

        self.camera_sub = self.create_subscription(Image, '/downscaled_image', self.image_callback, 10)
        self.debug_pub = self.create_publisher(Image, '/scan_code/debug', 10)

        self.image = None

        # scan twice (or more) to verify code is correct before publishing
        self.codes = [] # [['red', 'green', 'blue], ['red', 'green', 'blue']]
        self.is_black = False # code resets when it goes black

        self.create_timer(.1, self.read_code)

    def image_callback(self, msg:Image):
        self.image = msg
    
    def read_code(self):

        if self.image is None:
            return

        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        curr_code = read_curr_code(bgr)

        image_msg = CvBridge().cv2_to_imgmsg(curr_code, encoding='bgr8')
        self.get_logger().info('SENDING IMG')

        self.debug_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanCode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()