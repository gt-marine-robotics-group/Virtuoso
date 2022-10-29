import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from virtuoso_msgs.action import Translate

class TranslateActionServer(Node):

    def __init__(self):
        super().__init__('translate', namespace='navigation')

        self._action_server = ActionServer(self, Translate, 'translate', self.execute_callback)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing action')
        self.get_logger().info(f'{goal_handle.request.x}, {goal_handle.request.y}')

        goal_handle.succeed()
        result = Translate.Result()
        result.success = 1
        return result


def main(args=None):
    rclpy.init(args=args)

    translate_action_server = TranslateActionServer()

    rclpy.spin(translate_action_server)

if __name__ == '__main__':
    main()