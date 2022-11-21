import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class TestTranslate(Node):

    def __init__(self):
        super().__init__('test_translate', namespace='navigation')

        # self.action_client = ActionClient(self, Translate, '/navigation/translate')
        self.pub = self.create_publisher(Point, '/navigation/translate', 10)

        self.sent = False

        self.create_timer(1.0, self.send_goal)
    
    def send_goal(self):
        if self.sent: return
        self.get_logger().info('Sending Path')
        self.sent = True
        self.pub.publish(Point(x=100.0))


def main(args=None):
    rclpy.init(args=args)

    node = TestTranslate()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
