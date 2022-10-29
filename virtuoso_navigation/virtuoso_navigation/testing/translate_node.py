import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from virtuoso_msgs.action import Translate

class TestTranslate(Node):

    def __init__(self):
        super().__init__('test_translate', namespace='navigation')

        self.action_client = ActionClient(self, Translate, '/navigation/translate')
    
    def send_goal(self):
        goal_msg = Translate.Goal()
        goal_msg.x = 0.0
        goal_msg.y = 10.0

        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    test_translate = TestTranslate()

    future = test_translate.send_goal()

    rclpy.spin_until_future_complete(test_translate, future)