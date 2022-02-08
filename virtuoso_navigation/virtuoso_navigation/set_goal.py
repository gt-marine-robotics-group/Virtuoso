import rclpy
from rclpy.task import Future
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class SetGoal(Node):

    def __init__(self):
        super().__init__('set_goal')
        self.goal_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/set_goal', self.set_goal, 10)
        self.nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def set_goal(self, msg:PoseStamped):

        self.get_logger().info('getting goal')
        
        self.nav_action.wait_for_server()

        self.goal = NavigateToPose.Goal()
        self.goal.pose = msg

        self.get_logger().info('setting goal')

        self.future_goal = self.nav_action.send_goal_async(self.goal)

        self.future_goal.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future:Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future:Future):
        self.goal_accepted = False
        result = future.result()
        self.get_logger().info('Receieved result: ' + str(result))


def main(args=None):
    
    rclpy.init(args=args)

    node = SetGoal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()