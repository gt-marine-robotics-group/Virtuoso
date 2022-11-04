import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Int8
from cv_bridge import CvBridge
from .scan_code import ScanCode

class ScanCodeNode(Node):

    def __init__(self):
        super().__init__('perception_scan_code')

        # self.camera_sub = self.create_subscription(Image, '/downscaled_image', self.image_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.image_callback, 10)
        self.get_code_sub = self.create_subscription(Int8, '/perception/get_code', 
            self.start_scan, 10)

        self.ready_pub = self.create_publisher(Int8, '/perception/scan_code/ready', 10)
        self.code_pub = self.create_publisher(Int32MultiArray, '/perception/code', 10)

        self.debug_pub = self.create_publisher(Image, '/perception/debug', 10)

        self.declare_parameters(namespace='', parameters=[
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
        # self.scan_requested = True
        self.code_published = False

        self.scan_code = ScanCode(None, 10)
        self.scan_code.node = self

        self.create_timer(.1, self.read_code)
        self.create_timer(1.0, self.send_ready)
    
    def send_ready(self):
        if self.scan_requested:
            return
        msg = Int8()
        msg.data = 1
        self.ready_pub.publish(msg)
    
    def start_scan(self, msg):
        self.get_logger().info('Received Scan Code Request')
        self.scan_requested = True

    def image_callback(self, msg:Image):
        self.image = msg

    def read_code(self):

        if not self.scan_requested:
            return

        if self.code_published:
            return

        if self.image is None:
            return

        self.scan_code.image = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        self.scan_code.read_code()

        if self.scan_code.code is None:
            return
    
        self.code_published = True     
        self.get_logger().info(str(self.scan_code.code))
        msg = Int32MultiArray()
        msg.data = self.scan_code.code
        self.code_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanCodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()