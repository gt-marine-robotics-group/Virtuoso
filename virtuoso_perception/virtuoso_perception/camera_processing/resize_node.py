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
        ])

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

        resize = cv2.resize(bgr, (bgr.shape[1] // 2, bgr.shape[0] // 2), 
            interpolation=cv2.INTER_AREA)
        
        return self.cv_bridge.cv2_to_imgmsg(resize, encoding='bgr8')
    
    def resize_info(self, info:CameraInfo):
        info.k[0] /= 2
        info.k[2] /= 2
        info.k[4] /= 2
        info.k[5] /= 2
        info.width //= 2
        info.height //= 2
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