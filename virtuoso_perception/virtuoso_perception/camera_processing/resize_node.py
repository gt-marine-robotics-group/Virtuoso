import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class ResizeNode(Node):

    def __init__(self):
        super().__init__('perception_downscale')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
            ('resize_factor', 1)
        ])

        self.resize_factor = self.get_parameter('resize_factor').value

        base_topic = self.get_parameter('base_topic').value

        self.image_sub = self.create_subscription(Image, 
            f'{base_topic}/noise_filtered', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, 
            f'{base_topic}/camera_info', self.cam_info_callback, 10)
        
        self.resized_pub = self.create_publisher(Image,
            f'{base_topic}/resized', 10)
        self.resized_info_pub = self.create_publisher(CameraInfo, 
            f'{base_topic}/resized/camera_info', 10) 
        
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
    
    def image_callback(self, msg:Image):
        self.resized_pub.publish(self.resize(msg))
    
    def cam_info_callback(self, msg:CameraInfo):
        self.resized_info_pub.publish(self.resize_info(msg))


def main(args=None):
    
    rclpy.init(args=args)

    sub = ResizeNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()