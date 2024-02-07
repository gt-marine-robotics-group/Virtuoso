import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from virtuoso_msgs.msg import YOLOResultArray

class ChannelNavNode(Node):

    def __init__(self):
        super().__init__('channel_nav')

        self.control_mode_pub = self.create_publisher(String, 'controller_mode', 10)

        self.vel_pub = self.create_publisher(Twist, '/controller/manual/cmd_vel', 10)
        self.torque_pub = self.create_publisher(Float32, '/controller/manual/cmd_torque', 10)

        self.yolo_sub = self.create_subscription(YOLOResultArray, 'yolo_results', 
            self.yolo_callback, 10)

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.control_mode_is_set = False

        self.yolo_results = None
    
    def yolo_callback(self, msg: YOLOResultArray):
        self.yolo_results = msg 
    
    def timer_callback(self):
        if not self.control_mode_is_set:
            self.control_mode_pub.publish(String(data='manual'))
            self.control_mode_is_set = True
        
        if self.yolo_results is None:
            self.get_logger().info('No YOLO results')
            return
        
        self.get_logger().info(f'results: {self.yolo_results}')


def main(args=None):
    rclpy.init(args=args)

    node = ChannelNavNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()