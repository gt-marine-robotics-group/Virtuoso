import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import time
from virtuoso_msgs.action import ImageNoiseFilter

class NoiseFilterNode(Node):

    def __init__(self):
        super().__init__('perception_noise_filter')

        self.declare_parameters(namespace='', parameters=[
            ('debug', False),
            ('denoising_params', [])
        ])
        
        self.action_server = ActionServer(self, ImageNoiseFilter, 
            'perception/image_noise_filter', self.action_callback)

        self.cv_bridge = CvBridge()
    
    def action_callback(self, goal_handle):

        image:Image = goal_handle.request.image

        if image is None:
            goal_handle.abort()
            return

        bgr:np.ndarray = self.cv_bridge.imgmsg_to_cv2(image, 'bgr8')

        start_time = time.time()
        filtered = cv2.fastNlMeansDenoisingColored(bgr, None, 
            *self.get_parameter('denoising_params').value
        )
        if self.get_parameter('debug').value:
            self.get_logger().info(f'Noise execution time: {time.time() - start_time}')
        
        goal_handle.succeed()

        result = ImageNoiseFilter.Result()
        result.image = self.cv_bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
        return result

def main(args=None):
    
    rclpy.init(args=args)

    sub = NoiseFilterNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()