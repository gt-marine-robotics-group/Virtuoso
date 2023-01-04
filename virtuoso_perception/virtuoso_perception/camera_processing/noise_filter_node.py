import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import time

class NoiseFilterNode(Node):

    def __init__(self):
        super().__init__('perception_noise_filter')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
            ('debug', False),
            ('denoising_params', [])
        ])

        base_topic = self.get_parameter('base_topic').value

        self.image_sub = self.create_subscription(Image, 
            f'{base_topic}/image_raw', self.image_callback, 10)
        
        self.activate_sub = self.create_subscription(Bool, 
            f'/perception/camera/activate_processing', self.activate_callback, 10)

        self.filtered_pub = self.create_publisher(Image,
            f'{base_topic}/noise_filtered', 10)

        self.active = False
        
        self.cv_bridge = CvBridge()
    
    def activate_callback(self, msg:Bool):
        self.active = msg.data
    
    def image_callback(self, msg:Image):

        if not self.active:
            return

        bgr:np.ndarray = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        start_time = time.time()
        filtered = cv2.fastNlMeansDenoisingColored(bgr, None, 
            *self.get_parameter('denoising_params').value
        )
        if self.get_parameter('debug').value:
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