import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TestStopAllNode(Node):

    def __init__(self):
        super().__init__('test_ball_shooter_node')

        self.a_pub = self.create_publisher(Float32, '/ball_shooter/throttle_a_cmd', 10)

        self.b_pub = self.create_publisher(Float32, '/ball_shooter/throttle_b_cmd', 10) 

        self.water_pub = self.create_publisher(Float32, '/water_shooter/throttle_cmd', 10)
    
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Publishing command')
        self.a_pub.publish(Float32())
        self.b_pub.publish(Float32())
        self.water_pub.publish(Float32())


def main(args=None):
    rclpy.init(args=args)

    node = TestStopAllNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()