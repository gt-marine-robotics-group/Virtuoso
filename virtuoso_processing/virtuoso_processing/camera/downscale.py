import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage.transform import downscale_local_mean
import numpy as np

class DownScale(Node):

    def __init__(self):
        super().__init__('downscale')
        self.grayscaled_sub = self.create_subscription(Image, 'grayscaled_image', self.listener_callback, 10) # subscribes to topic published to by grayscale node
        self.publisher = self.create_publisher(Image, 'downscaled_image', 10)
    
    def listener_callback(self, msg:Image):
        data = CvBridge().imgmsg_to_cv2(msg, 'mono8')

        downscaled:np.ndarray = downscale_local_mean(data, (10, 10))
        downscaled = downscaled.astype('uint8')

        returnMsg = CvBridge().cv2_to_imgmsg(downscaled, 'mono8')
        returnMsg.header.frame_id = 'downscaled'

        self.publisher.publish(returnMsg)


def main(args=None):
    
    rclpy.init(args=args)

    sub = DownScale()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()