import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from skimage.transform import downscale_local_mean
import numpy as np

class DownScale(Node):

    def __init__(self):
        super().__init__('processing_downscale')
        self.subscription = self.create_subscription(Image,
            '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.listener_callback, 10) # subscribes to topic for raw camera data
        self.publisher = self.create_publisher(Image, '/processing/image/downscaled', 10)

        self.g1_sub = self.create_subscription(Image, '/processing/image/grayscaled1', 
            self.g1_callback, 10)
        self.g2_sub = self.create_subscription(Image, '/processing/image/grayscaled2',
            self.g2_callback, 10)
        
        self.d1_pub = self.create_publisher(Image, '/processing/image/downscaled1', 10)
        self.d2_pub = self.create_publisher(Image, '/processing/image/downscaled2', 10)

        self.d1_info_pub = self.create_publisher(CameraInfo, 
            '/processing/image/downscaled1/camera_info', 10)
        self.d2_info_pub = self.create_publisher(CameraInfo, 
            '/processing/image/downscaled2/camera_info', 10)
    
    def listener_callback(self, msg:Image):
        self.publisher.publish(msg)
    
    def downscale(self, img:Image):
        data = CvBridge().imgmsg_to_cv2(img, 'mono8')

        downscaled = downscale_local_mean(data, (2, 2)).astype('uint8')

        return CvBridge().cv2_to_imgmsg(downscaled, 'mono8')
    
    # def downscale_info
    
    def g1_callback(self, msg:Image):
        self.d1_pub.publish(self.downscale(msg))
    
    def g2_callback(self, msg:Image):
        self.d2_pub.publish(self.downscale(msg))


def main(args=None):
    
    rclpy.init(args=args)

    sub = DownScale()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()