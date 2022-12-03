import rclpy
from rclpy.node import Node
from virtuoso_perception.utils.ColorFilter import ColorFilter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BuoyColorFilterNode(Node):

    def __init__(self):
        super().__init__('processing_buoy_color_filter')

        self.image1_sub = self.create_subscription(Image, 
            '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.image1_callback, 10)

        self.image2_sub = self.create_subscription(Image,
            '/wamv/sensors/cameras/front_right_camera/image_raw',
            self.image2_callback, 10)
        
        self.bc_filter1_pub = self.create_publisher(Image,
            '/processing/image/buoy_color_filter1', 10)
        self.bc_filter2_pub = self.create_publisher(Image,
            '/processing/image/buoy_color_filter2', 10)
    
    def apply_filter(self, img:Image):
        bgr_image = CvBridge().imgmsg_to_cv2(img)

        self.color_filter = ColorFilter(cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV), bgr_image)

        blue_filtered = self.color_filter.blue_filter()

        return CvBridge().cv2_to_imgmsg(blue_filtered, encoding='bgr8')
    
    def image1_callback(self, msg:Image):
        self.bc_filter1_pub.publish(self.apply_filter(msg))
    
    def image2_callback(self, msg:Image):
        self.bc_filter2_pub.publish(self.apply_filter(msg))


def main(args=None):
    
    rclpy.init(args=args)

    sub = BuoyColorFilterNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()