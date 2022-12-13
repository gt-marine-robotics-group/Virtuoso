import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

# class DownScale(Node):

#     def __init__(self):
#         super().__init__('perception_downscale')
#         self.subscription = self.create_subscription(Image,
#             '/wamv/sensors/cameras/front_left_camera/image_raw', 
#             self.listener_callback, 10) # subscribes to topic for raw camera data
#         self.publisher = self.create_publisher(Image, '/perception/image/downscaled', 10)

#         self.g1_sub = self.create_subscription(Image, '/perception/image/grayscaled1', 
#             self.g1_callback, 10)
#         self.g2_sub = self.create_subscription(Image, '/perception/image/grayscaled2',
#             self.g2_callback, 10)

#         self.cam_info1_sub = self.create_subscription(CameraInfo, 
#             '/wamv/sensors/cameras/front_left_camera/camera_info', self.cam_info1_callback, 10)
#         self.cam_info2_sub = self.create_subscription(CameraInfo,
#             '/wamv/sensors/cameras/front_right_camera/camera_info', self.cam_info2_callback, 10)
        
#         self.d1_pub = self.create_publisher(Image, '/perception/image/downscaled1', 10)
#         self.d2_pub = self.create_publisher(Image, '/perception/image/downscaled2', 10)

#         self.d1_info_pub = self.create_publisher(CameraInfo, 
#             '/perception/image/downscaled1/camera_info', 10)
#         self.d2_info_pub = self.create_publisher(CameraInfo, 
#             '/perception/image/downscaled2/camera_info', 10)
    
#     def listener_callback(self, msg:Image):
#         self.publisher.publish(msg)
    
#     def downscale(self, img:Image):
#         data = CvBridge().imgmsg_to_cv2(img, 'mono8')

#         downscaled = downscale_local_mean(data, (2, 2)).astype('uint8')

#         return CvBridge().cv2_to_imgmsg(downscaled, 'mono8')
    
#     def downscale_info(self, info:CameraInfo):
#         info.k[0] /= 2
#         info.k[2] /= 2
#         info.k[4] /= 2
#         info.k[5] /= 2
#         return info
    
#     def cam_info1_callback(self, msg:CameraInfo):
#         self.d1_info_pub.publish(self.downscale_info(msg))
    
#     def cam_info2_callback(self, msg:CameraInfo):
#         self.d2_info_pub.publish(self.downscale_info(msg))
    
#     def g1_callback(self, msg:Image):
#         self.d1_pub.publish(self.downscale(msg))
    
#     def g2_callback(self, msg:Image):
#         self.d2_pub.publish(self.downscale(msg))

class ResizeNode(Node):

    def __init__(self):
        super().__init__('perception_downscale')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
        ])

        base_topic = self.get_parameter('base_topic').value

        self.image_sub = self.create_subscription(Image, 
            f'{base_topic}/image_raw', self.image_callback, 10)
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