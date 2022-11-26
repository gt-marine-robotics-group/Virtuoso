import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage.transform import downscale_local_mean, resize
import numpy as np

class DownScale(Node):

    def __init__(self):
        super().__init__('processing_downscale')
        self.subscription = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', self.listener_callback, 10) # subscribes to topic for raw camera data
        self.publisher = self.create_publisher(Image, '/processing/image/downscaled', 10)
    
    def listener_callback(self, msg:Image):

        self.publisher.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    sub = DownScale()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()