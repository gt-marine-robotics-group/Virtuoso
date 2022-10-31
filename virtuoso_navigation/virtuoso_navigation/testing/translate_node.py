import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class TestTranslate(Node):

    def __init__(self):
        super().__init__('test_translate', namespace='navigation')

        # self.action_client = ActionClient(self, Translate, '/navigation/translate')
        self.pub = self.create_publisher(Point, '/navigation/translate', 10)
    
    def send_goal(self):
        self.pub.publish(Point(y=2.0))


def main(args=None):
    rclpy.init(args=args)

    test_translate = TestTranslate()

    future = test_translate.send_goal()

    rclpy.spin_until_future_complete(test_translate, future)