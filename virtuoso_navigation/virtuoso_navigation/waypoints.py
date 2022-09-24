from math import sqrt
import rclpy
from rclpy import Future
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient

class Waypoints(Node):

    def __init__(self):
        super().__init__('waypoint_nav')

        self.goal_sub = self.create_subscription(Path, '/virtuoso_navigation/set_path', self.set_path, 10)
        self.nav_action = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)

        self.waypoint_pub = self.create_publisher(Odometry, '/waypoint_manual', 10)

        self.success_pub = self.create_publisher(PoseStamped, '/virtuoso_navigation/success', 10)

        self.waypoints_completed = 0
        self.path = None
        self.nav2_path = None
        self.nav2_waypoints_completed = 0
        self.nav2_goal = None
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

        self.nav2_path = None
        self.nav2_waypoints_completed = 0
        self.nav2_goal = None
    
    def calc_nav2_path(self):
        self.get_logger().info('sending nav2 goal')
        self.nav2_goal = ComputePathToPose.Goal()

        self.nav2_goal.pose = PoseStamped()
        self.nav2_goal.pose.pose = self.path.poses[self.waypoints_completed].pose

        goal_future = self.nav_action.send_goal_async(self.nav2_goal)
        goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            # send straight path
            return
        self.get_logger().info('goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_goal_done_callback) 
    
    def nav2_goal_done_callback(self, future):
        result = future.result()

        self.nav2_path = result.result.path
    
    def find_next_nav2_waypoint(self):
        if self.nav2_waypoints_completed == len(self.nav2_path.poses) - 1:
            return self.nav2_waypoints_completed + 1
        curr = self.nav2_path.poses[self.nav2_waypoints_completed].pose
        next_point = self.nav2_waypoints_completed + 1
        while next_point < len(self.nav2_path.poses):
            if self.distance(curr, self.nav2_path.poses[next_point].pose) >= 10:
                break
            next_point += 1
        if next_point == len(self.nav2_path.poses):
            return next_point - 1
        return next_point
    
    def navigate(self):

        if self.robot_pose is None or self.path is None:
            return

        if not self.nav2_path:
            if not self.nav2_goal:
                self.calc_nav2_path() 
            return        

        if self.distance(self.robot_pose, self.nav2_path.poses[self.nav2_waypoints_completed].pose) < 1:
            self.nav2_waypoints_completed = self.find_next_nav2_waypoint()
        
        if self.nav2_waypoints_completed == len(self.nav2_path.poses):
            self.waypoints_completed += 1
            if self.waypoints_completed == len(self.path.poses):
                self.get_logger().info('COMPLETED GOAL')
                self.success_pub.publish(self.path.poses[self.waypoints_completed - 1])
                self.path = None
                return
            self.nav2_path = None
            self.nav2_goal = None
            return

        self.send_waypoint(self.nav2_path.poses[self.nav2_waypoints_completed].pose)
    
    def send_waypoint(self, pose):
        msg = Odometry()
        msg.header.frame_id = 'map'
        msg.pose.pose = pose
        msg.pose.pose.orientation = self.path.poses[self.waypoints_completed].pose.orientation

        self.waypoint_pub.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    node = Waypoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
