from skimage.color import rgb2gray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class GrayScale(Node):

    def __init__(self):
        super().__init__('perception_grayscale')

        # self.g1_sub = self.create_subscription(Image, 
        #     '/wamv/sensors/cameras/front_left_camera/image_raw',
        #     self.g1_callback, 10) # subscribes to topic for raw camera data
        self.g1_sub = self.create_subscription(Image,
            '/perception/buoys/buoy_color_filter1', self.g1_callback, 10)

        # self.g2_sub = self.create_subscription(Image,
        #     '/wamv/sensors/cameras/front_right_camera/image_raw',
        #     self.g2_callback, 10)
        self.g2_sub = self.create_subscription(Image,
            '/perception/buoys/buoy_color_filter2', self.g2_callback, 10)

        self.g1_publisher = self.create_publisher(Image, '/perception/image/grayscaled1', 10)
        self.g2_publisher = self.create_publisher(Image, '/perception/image/grayscaled2', 10)
    
    def grayscale(self, img:Image):
        rgbData = CvBridge().imgmsg_to_cv2(img_msg=img, desired_encoding='rgb8')

        grayscale:np.ndarray = rgb2gray(rgbData)
        grayscale *= 255
        grayscale = grayscale.astype('uint8')

        msg = CvBridge().cv2_to_imgmsg(grayscale, encoding='mono8') 
        msg.header.frame_id = img.header.frame_id

        return msg
    
    def g1_callback(self, msg:Image):
        self.g1_publisher.publish(self.grayscale(msg)) 

    def g2_callback(self, msg:Image):
        self.g2_publisher.publish(self.grayscale(msg))


def main(args=None):
    rclpy.init(args=args)

    sub = GrayScale()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()