import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Path
from std_msgs.msg import Int8, Int32MultiArray
from .find_docks import FindDocks

class FindDocksNode(Node):

    def __init__(self):
        super().__init__('find_docks', namespace='perception')

        self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.image_callback, 10)
        self.start_sub = self.create_subscription(Int8, '/perception/start_find_docks', 
            self.start_callback, 10)

        self.ready_pub = self.create_publisher(Int8, 'find_docks/ready', 10)
        
        # [Red dock offset, Green dock offset, Blue dock offset]
        # e.g. [100, 0, -100] -> red dock is 100px left, green dock staight in front,
        # blue dock 100px right
        self.dock_info_pub = self.create_publisher(Int32MultiArray, 
            '/perception/dock_code_offsets', 10)

        self.find_docks = FindDocks() 

        self.create_timer(.1, self.find)
        self.create_timer(1.0, self.send_ready)
    
    def image_callback(self, image):
        self.find_docks.image = image
    
    def start_callback(self, msg):
        self.find_docks.search_requested = True
    
    def send_ready(self):
        msg = self.find_docks.get_ready_msg()
        if msg is None:
            return
        self.ready_pub.publish(msg)
    
    def find(self):
        self.find_docks.find(self)

def main(args=None):
    rclpy.init(args=args)
    node = FindDocksNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()