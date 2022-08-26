import rclpy
from rclpy import Future
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class Waypoints(Node):

    def __init__(self):
        super().__init__('waypoint_nav')

        self.goal_sub = self.create_subscription(Path, '/virtuoso_navigation/set_path', self.set_path, 10)
        self.nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.waypoints_completed = 0
        self.path = None
        self.goal = None
    
    def set_path(self, msg:Path):

        self.get_logger().info('setting path')

        self.path = msg
        self.waypoints_completed = 0

        self.nav_to_next_waypoint()

    def nav_to_next_waypoint(self):

        if (self.waypoints_completed >= len(self.path.poses)): return

        self.goal = NavigateToPose.Goal()
        self.goal.pose = self.path.poses[self.waypoints_completed]
        #self.goal.behavior_tree = "/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery"
        
        goal_future = self.nav_action.send_goal_async(self.goal)

        goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future:Future):
        goal_handle = future.result()
        if (not goal_handle.accepted):
            self.get_logger().info('goal rejected')
            return
        self.get_logger().info('goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_done_callback)
    
    def goal_done_callback(self, future:Future):
        result = future.result()
        self.get_logger().info('successfully navigated to pose ' + str(self.waypoints_completed))

        if (result.status == 4):
            self.waypoints_completed += 1
            
        self.nav_to_next_waypoint()



def main(args=None):
    
    rclpy.init(args=args)

    node = Waypoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
