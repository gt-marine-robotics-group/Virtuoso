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
        self.goal_accepted = False

    def set_goal(self, msg:PoseStamped):

        self.get_logger().info('getting goal')
        
        self.nav_action.wait_for_server()

        if not self.goal_accepted:
            self.set_new_goal(msg)
            return
        
        self.change_goal(msg)

    def change_goal(self, msg):
        self.get_logger().info('cancelling goal')
        self.cancel = self.nav_action._cancel_goal_async(self.future_goal.result())
        self.cancel.add_done_callback(self.set_new_goal(msg))

    def set_new_goal(self, msg:PoseStamped):
        self.goal = NavigateToPose.Goal()
        self.goal.pose = msg

        self.get_logger().info('setting goal')

        self.future_goal = self.nav_action.send_goal_async(self.goal, feedback_callback=self.feedback_callback)

        self.future_goal.add_done_callback(self.goal_response_callback)

    
    def feedback_callback(self, msg):
        # self.get_logger().info(str(msg))
        pass
    
    def goal_response_callback(self, future:Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.goal_accepted = True
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