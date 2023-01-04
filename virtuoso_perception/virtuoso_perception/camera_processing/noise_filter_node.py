import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import time

class NoiseFilterNode(Node):

    def __init__(self):
        super().__init__('perception_noise_filter')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
        ])

        base_topic = self.get_parameter('base_topic').value

        self.image_sub = self.create_subscription(Image, 
            f'{base_topic}/image_raw', self.image_callback, 10)

        self.filtered_pub = self.create_publisher(Image,
            f'{base_topic}/noise_filtered', 10)
        
        self.cv_bridge = CvBridge()
    
    def image_callback(self, msg:Image):
        bgr:np.ndarray = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        start_time = time.time()
        filtered = cv2.fastNlMeansDenoisingColored(bgr, None, 20, 20, 7, 21)
        self.get_logger().info(f'Noise execution time: {time.time() - start_time}')

        self.filtered_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            filtered, encoding='bgr8'))

def main(args=None):
    
    rclpy.init(args=args)

    sub = NoiseFilterNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()