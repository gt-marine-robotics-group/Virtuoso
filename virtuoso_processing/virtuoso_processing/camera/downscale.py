import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage.transform import downscale_local_mean, resize
import numpy as np

class DownScale(Node):

    def __init__(self):
        super().__init__('downscale')
        # self.subscription = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', self.listener_callback, 10) # subscribes to topic for raw camera data
        # self.publisher = self.create_publisher(Image, 'downscaled_image', 10)
    
    def listener_callback(self, msg:Image):
        data = CvBridge().imgmsg_to_cv2(msg, 'rgb8')

        resized = resize(data, (144, 256), preserve_range=True) # original size (720, 1280)

        returnMsg = CvBridge().cv2_to_imgmsg(np.uint8(resized), 'rgb8')

        self.publisher.publish(returnMsg)


def main(args=None):
    
    rclpy.init(args=args)

    sub = DownScale()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()