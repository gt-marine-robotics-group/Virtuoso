import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class TestManuelYawNode(Node):

    def __init__(self):
        super().__init__('controller_test_manual_yaw')

        self.control_mode_pub = self.create_publisher(String, 'controller_mode', 10)

        self.vel_pub = self.create_publisher(Twist, '/controller/manual/cmd_vel', 10)
        self.torque_pub = self.create_publisher(Float32, '/controller/manual/cmd_torque', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Publishing')

        mode = String()
        mode.data = 'manual'
        self.control_mode_pub.publish(mode)

        torque = Float32()
        torque.data = 1.0

        self.vel_pub.publish(Twist())
        self.torque_pub.publish(torque)


def main(args=None):
    rclpy.init(args=args)

    node = TestManuelYawNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()