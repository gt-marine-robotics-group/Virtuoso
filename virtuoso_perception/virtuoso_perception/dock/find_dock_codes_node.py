import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Path
from std_msgs.msg import Int8, Int32MultiArray
from .find_dock_codes import FindDockCodes

class FindDocksNode(Node):

    def __init__(self):
        super().__init__('perception_find_dock_codes')

        self.camera_sub = self.create_subscription(Image, '/processing/image/downscaled', 
            self.image_callback, 10)
        self.start_sub = self.create_subscription(Int8, '/perception/start_find_docks', 
            self.start_callback, 10)

        self.ready_pub = self.create_publisher(Int8, '/perception/find_dock_codes/ready', 10)
        
        # [Red dock offset, Green dock offset, Blue dock offset, Red unknown, Green unknown, 
        #   Blue unknown, image width]
        # e.g. [100, 0, -100, 0, 0, 0] -> red dock is 100px left, green dock staight in front,
        # blue dock 100px right
        self.dock_info_pub = self.create_publisher(Int32MultiArray, 
            '/perception/dock_code_offsets', 10)

        self.declare_parameters(namespace='', parameters=[
            ('red.lower1', [0,0,0]),
            ('red.upper1', [0,0,0]),
            ('red.lower2', [0,0,0]),
            ('red.upper2', [0,0,0]),
            ('green.lower', [0,0,0]),
            ('green.upper', [0,0,0]),
            ('blue.lower', [0,0,0]),
            ('blue.upper', [0,0,0]),
            
            ('code_axis_range', 0.0),
            ('debug', False)
        ])

        self.image = None
        self.search_requested = False

        self.find_docks = FindDockCodes(filter_bounds={
            'red': {
                'lower1': self.get_parameter('red.lower1').value,
                'upper1': self.get_parameter('red.upper1').value,
                'lower2': self.get_parameter('red.lower2').value,
                'upper2': self.get_parameter('red.upper2').value
            },
            'green': {
                'lower': self.get_parameter('green.lower').value,
                'upper': self.get_parameter('green.upper').value
            },
            'blue': {
                'lower': self.get_parameter('blue.lower').value,
                'upper': self.get_parameter('blue.upper').value
            }
        }, code_axis_range=self.get_parameter('code_axis_range').value)
        if self.get_parameter('debug').value:
            self.find_docks.node = self

        self.create_timer(.1, self.find)
    
    def image_callback(self, image):
        self.image = image
    
    def start_callback(self, msg):
        self.search_requested = True
    
    def find(self):
        if self.search_requested and self.image:
            self.find_docks.image = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        offsets = self.find_docks.find_docks()
        if offsets is None:
            return
        msg = Int32MultiArray()
        msg.data = list(int(offset) for offset in offsets)
        self.dock_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FindDocksNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()