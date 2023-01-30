import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2
from virtuoso_msgs.action import ResizeImage

class ResizeNode(Node):

    def __init__(self):
        super().__init__('perception_downscale')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
            ('resize_factor', 1)
        ])

        self.action_server = ActionServer(self, ResizeImage, 
            'perception/image_resize', self.action_callback)

        self.resize_factor = self.get_parameter('resize_factor').value

        self.cv_bridge = CvBridge()

    def resize(self, img:Image):
        bgr:np.ndarray = self.cv_bridge.imgmsg_to_cv2(img, 'bgr8')

        resize = cv2.resize(bgr, (bgr.shape[1] // self.resize_factor, 
            bgr.shape[0] // self.resize_factor), 
            interpolation=cv2.INTER_AREA
        )
        
        return self.cv_bridge.cv2_to_imgmsg(resize, encoding='bgr8')
    
    def resize_info(self, info:CameraInfo):
        info.k[0] /= self.resize_factor
        info.k[2] /= self.resize_factor
        info.k[4] /= self.resize_factor
        info.k[5] /= self.resize_factor
        info.width //= self.resize_factor
        info.height //= self.resize_factor
        return info
    
    def action_callback(self, goal_handle):
        image:Image = goal_handle.request.image
        camera_info:CameraInfo = goal_handle.request.camera_info

        if image is None or camera_info is None:
            goal_handle.abort()
            return
        
        image = self.resize(image)
        camera_info = self.resize_info(camera_info)

        result = ResizeImage.Result()
        result.image = image
        result.camera_info = camera_info
        return result


def main(args=None):
    
    rclpy.init(args=args)

    sub = ResizeNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()