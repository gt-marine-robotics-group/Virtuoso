import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2
from virtuoso_msgs.srv import ImageResize
from .resize import Resize

class ResizeNode(Node):

    def __init__(self):
        super().__init__('perception_downscale')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
            ('resize_factor', 1)
        ])

        base_topic = self.get_parameter('base_topic').value
        cam = base_topic[base_topic.rfind('/') + 1:]

        self.srv = self.create_service(ImageResize, 
            f'{cam}/resize', self.srv_callback)

        self.resize = Resize(resize_factor=self.get_parameter('resize_factor').value)

    def srv_callback(self, req:ImageResize.Request, res:ImageResize.Response):
        image:Image = req.image
        camera_info:CameraInfo = req.camera_info

        if image is None or camera_info is None:
            return res
        
        self.resize.image = image
        self.resize.camera_info = camera_info
        image, camera_info = self.resize.resize()

        res.image = image
        res.camera_info = camera_info
        return res


def main(args=None):
    
    rclpy.init(args=args)

    sub = ResizeNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()