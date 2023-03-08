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
from virtuoso_msgs.srv import ImageNoiseFilter
from .noise_filter import NoiseFilter

class NoiseFilterNode(Node):

    def __init__(self):
        super().__init__('perception_noise_filter')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
            ('debug', False),
            ('denoising_params', [])
        ])

        base_topic = self.get_parameter('base_topic').value
        cam = base_topic[base_topic.rfind('/') + 1:]
        
        self.srv = self.create_service(ImageNoiseFilter, 
            f'{cam}/noise_filter', self.srv_callback)

        self.filter = NoiseFilter(self.get_parameter('denoising_params').value)
    
    def srv_callback(self, req:ImageNoiseFilter.Request, 
        res:ImageNoiseFilter.Response):

        image = req.image

        if image is None:
            res.image = None
            return res
        
        self.filter.image = image

        start_time = time.time()
        image = self.filter.filter()
        if self.get_parameter('debug').value:
            self.get_logger().info(f'Noise execution time: {time.time() - start_time}')
        
        res.image = image
        return res

def main(args=None):
    
    rclpy.init(args=args)

    sub = NoiseFilterNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()