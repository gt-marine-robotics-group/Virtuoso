import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Path
from std_msgs.msg import Int8, Int32MultiArray

class FindDocks(Node):

    def __init__(self):
        super().__init__('find_docks')

        self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.image_callback, 10)
        self.start_sub = self.create_subscription(Int8, '/perception/start_find_docks', 
            self.start_callback, 10)

        self.ready_pub = self.create_publisher(Int8, '/perception/find_docks/ready', 10)
        
        # [Red dock offset, Green dock offset, Blue dock offset]
        # e.g. [100, 0, -100] -> red dock is 100px left, green dock staight in front,
        # green dock 100px right
        self.dock_info_pub = self.create_publisher(Int32MultiArray, 
            '/perception/dock_code_offsets', 10)
        
        self.image = None

        # self.search_requested = False
        self.search_requested = True

        self.create_timer(.1, self.find_docks)
        self.create_timer(1.0, self.send_ready)
    
    def image_callback(self, image):
        self.image = image
    
    def start_callback(self, msg):
        self.search_requested = True
    
    def send_ready(self):
        if self.search_requested:
            return
        msg = Int8()
        msg.data = 1
        self.ready_pub.publish(msg)
    
    def find_docks(self):

        if not self.search_requested:
            return
        
        if self.image is None:
            return
        
        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        shape = bgr.shape
        self.get_logger().info(f'Shape: {shape}')


def main(args=None):
    rclpy.init(args=args)
    node = FindDocks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()