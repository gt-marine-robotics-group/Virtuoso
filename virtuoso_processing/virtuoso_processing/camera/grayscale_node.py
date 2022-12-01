from skimage.color import rgb2gray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class GrayScale(Node):

    def __init__(self):
        super().__init__('grayscale')
        self.subscription = self.create_subscription(Image, 
            '/wamv/sensors/cameras/front_left_camera/image_raw',
            self.listener_callback, 10) # subscribes to topic for raw camera data
        self.publisher = self.create_publisher(Image, '/processing/image/grayscaled', 10)
    
    def listener_callback(self, msg:Image):
        rgbData = CvBridge().imgmsg_to_cv2(img_msg=msg, desired_encoding='rgb8')

        grayscale:np.ndarray = rgb2gray(rgbData)
        grayscale *= 255
        grayscale = grayscale.astype('uint8')

        returnMsg = CvBridge().cv2_to_imgmsg(grayscale, encoding='mono8') 

        self.publisher.publish(returnMsg) 


def main(args=None):
    rclpy.init(args=args)

    sub = GrayScale()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()