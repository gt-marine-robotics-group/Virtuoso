from math import sqrt
import rclpy
from rclpy import Future
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient

class Waypoints(Node):

    def __init__(self):
        super().__init__('waypoint_nav')

        self.goal_sub = self.create_subscription(Path, '/virtuoso_navigation/set_path', self.set_path, 10)
        # self.nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)

        self.waypoint_pub = self.create_publisher(Odometry, '/waypoint_manual', 10)

        self.success_pub = self.create_publisher(PoseStamped, '/virtuoso_navigation/success', 10)

        self.waypoints_completed = 0
        self.path = None
        self.robot_pose = None

        self.create_timer(.1, self.navigate)
    
    def odom_callback(self, odom:Odometry):
        self.robot_pose = odom.pose.pose

    def distance(self, p1, p2):
        return sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)
    
    def set_path(self, msg:Path):

        self.get_logger().info('setting path')

        self.path = msg
        self.waypoints_completed = 0
    
    def navigate(self):

        if self.path is None:
            return

        if self.distance(self.robot_pose, self.path.poses[self.waypoints_completed].pose) < 1:
            self.waypoints_completed += 1

        if self.waypoints_completed == len(self.path.poses):
            self.get_logger().info('COMPLETED GOAL')
            self.success_pub.publish(self.path.poses[self.waypoints_completed - 1])
            self.path = None
            return

        msg = Odometry()
        msg.header.frame_id = 'map'
        msg.pose.pose = self.path.poses[self.waypoints_completed].pose

        self.waypoint_pub.publish(msg)

    # def nav_to_next_waypoint(self):

    #     if (self.waypoints_completed >= len(self.path.poses)):
    #         self.success_pub.publish(self.path.poses[-1])
    #         return

    #     self.goal = NavigateToPose.Goal()
    #     self.goal.pose = self.path.poses[self.waypoints_completed]
    #     self.goal.behavior_tree = "/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_time.xml"
        
    #     goal_future = self.nav_action.send_goal_async(self.goal)

    #     goal_future.add_done_callback(self.goal_response_callback)
    
    # def goal_response_callback(self, future:Future):
    #     goal_handle = future.result()
    #     if (not goal_handle.accepted):
    #         self.get_logger().info('goal rejected')
    #         return
    #     self.get_logger().info('goal accepted')

    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.goal_done_callback)
    
    # def goal_done_callback(self, future:Future):
    #     result = future.result()
    #     self.get_logger().info('successfully navigated to pose ' + str(self.waypoints_completed))

    #     if (result.status == 4):
    #         self.waypoints_completed += 1
            
    #     self.nav_to_next_waypoint()



def main(args=None):
    
    rclpy.init(args=args)

    node = Waypoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
