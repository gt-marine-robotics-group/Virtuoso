import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

CMD = 1.0

class TestWaterShooter(Node):

    def __init__(self):
        super().__init__('test_ball_shooter_node')

        self.cmd_pub = self.create_publisher(Float32, '/water_shooter/throttle_cmd', 10)
    
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Publishing command')
        self.cmd_pub.publish(Float32(data=CMD))


def main(args=None):
    rclpy.init(args=args)

    node = TestWaterShooter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()