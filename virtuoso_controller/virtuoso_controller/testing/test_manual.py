import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class TestManuelNode(Node):

    def __init__(self):
        super().__init__('controller_test_manual')

        self.control_mode_pub = self.create_publisher(String, 'controller_mode', 10)

        self.vel_pub = self.create_publisher(Twist, '/controller/manual/cmd_vel', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Publishing')

        mode = String()
        mode.data = 'manual'
        self.control_mode_pub.publish(mode)

        twist = Twist()
        twist.linear.x = 1.0

        self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = TestManuelNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()