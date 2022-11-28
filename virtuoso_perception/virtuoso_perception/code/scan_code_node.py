import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Int8
from cv_bridge import CvBridge
from .scan_code import ScanCode

class ScanCodeNode(Node):

    def __init__(self):
        super().__init__('perception_scan_code')

        self.camera_sub = self.create_subscription(Image, '/processing/image/downscaled', 
            self.image_callback, 10)
        self.get_code_sub = self.create_subscription(Int8, '/perception/get_code', 
            self.start_scan, 10)

        self.ready_pub = self.create_publisher(Int8, '/perception/scan_code/ready', 10)
        self.code_pub = self.create_publisher(Int32MultiArray, '/perception/code', 10)

        self.debug_find_code_coord_pub = self.create_publisher(Image, 
            '/perception/debug/find_code_coord', 10)
        self.debug_red_filter_pub = self.create_publisher(Image,
            '/perception/debug/code_red_filter', 10)
        self.debug_green_filter_pub = self.create_publisher(Image,
            '/perception/debug/code_green_filter', 10)
        self.debug_blue_filter_pub = self.create_publisher(Image,
            '/perception/debug/code_blue_filter', 10)

        self.debugs = {
            'find_code_coord': self.debug_find_code_coord,
            'code_red_filter': self.debug_code_red_filter,
            'code_green_filter': self.debug_code_green_filter,
            'code_blue_filter': self.debug_code_blue_filter
        }

        self.declare_parameters(namespace='', parameters=[
            ('debug', False),
            ('code_loc_noise', 0.0),

            ('red.lower1', [0,0,0]),
            ('red.upper1', [0,0,0]),
            ('red.lower2', [0,0,0]),
            ('red.upper2', [0,0,0]),
            ('green.lower', [0,0,0]),
            ('green.upper', [0,0,0]),
            ('blue.lower', [0,0,0]),
            ('blue.upper', [0,0,0])
        ])

        self.image = None
        self.scan_requested = False
        self.code_published = False

        self.scan_code = ScanCode(filter_bounds={
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
        }, code_loc_noise=10)
        if self.get_parameter('debug').value:
            self.scan_code.node = self

        self.create_timer(.1, self.read_code)
    
    def start_scan(self, msg):
        self.get_logger().info('Received Scan Code Request')
        self.scan_requested = True

    def image_callback(self, msg:Image):
        self.image = msg

    def read_code(self):

        if not self.scan_requested:
            return

        if self.image is None:
            return

        self.scan_code.image = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        self.scan_code.filter_colors()

        if self.code_published:
            return

        self.scan_code.read_code()

        if self.scan_code.code is None:
            return
    
        self.code_published = True     
        self.get_logger().info(str(self.scan_code.code))
        msg = Int32MultiArray()
        msg.data = self.scan_code.code
        self.code_pub.publish(msg)
    
    def cv_to_imgmsg(self, cv_image):
        return CvBridge().cv2_to_imgmsg(cv_image, encoding='bgr8')
    
    def debug_find_code_coord(self, cv_image):
        self.debug_find_code_coord_pub.publish(self.cv_to_imgmsg(cv_image))
    
    def debug_code_red_filter(self, cv_image):
        self.debug_red_filter_pub.publish(self.cv_to_imgmsg(cv_image))
    
    def debug_code_blue_filter(self, cv_image):
        self.debug_blue_filter_pub.publish(self.cv_to_imgmsg(cv_image))

    def debug_code_green_filter(self, cv_image):
        self.debug_green_filter_pub.publish(self.cv_to_imgmsg(cv_image))

def main(args=None):
    rclpy.init(args=args)
    node = ScanCodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()