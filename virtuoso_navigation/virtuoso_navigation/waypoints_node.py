from math import sqrt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from std_msgs.msg import Bool
import math

class Waypoints(Node):

    def __init__(self):
        super().__init__('navigation_waypoints')

        self.goal_sub = self.create_subscription(Path, '/navigation/set_path', self.set_path, 10)
        self.translate_sub = self.create_subscription(Path, '/navigation/set_trans_path', 
            self.set_trans_path, 10)
        self.nav_action = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)

        self.success_pub = self.create_publisher(PoseStamped, '/navigation/success', 10)
        
        self.path_pub = self.create_publisher(Path, '/navigation/plan', 10)
        self.is_trans_pub = self.create_publisher(Bool, '/controller/is_translation', 10)

        self.declare_parameters(namespace='', parameters=[
            ('use_nav2', True),
            ('only_translate', False),
            ('goal_dist_tolerance', 0.0)
        ])

        self.waypoints_completed = 0
        self.path = None
        self.nav2_path = None
        self.nav2_goal = None
        self.robot_pose = None

        self.create_timer(.1, self.navigate)
    
    def odom_callback(self, odom:Odometry):
        self.robot_pose = odom.pose.pose

    def distance(self, p1, p2):
        return sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)
    
    def set_path(self, msg:Path, is_trans=False):

        self.get_logger().info('Setting Path')

        self.path = msg
        self.waypoints_completed = 0

        self.nav2_path = None
        self.nav2_goal = None

        self.is_trans_pub.publish(
            Bool(data= is_trans or self.get_parameter('only_translate').value)
        )

    def set_trans_path(self, msg:Path):
        self.set_path(msg, True) 
    
    def calc_nav2_path(self):
        self.nav2_goal = ComputePathToPose.Goal()

        if not self.get_parameter('use_nav2').value:
            self.get_logger().info('Creating Straight path')
            self.nav2_path = self.create_straight_path()
            return

        self.nav2_goal.pose = PoseStamped()
        self.nav2_goal.pose.pose = self.path.poses[self.waypoints_completed].pose

        self.get_logger().info('Sending Nav2 goal')

        goal_future = self.nav_action.send_goal_async(self.nav2_goal)
        goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav2 Goal Rejected: Creating straight path')
            self.nav2_path = self.create_straight_path()
            return
        self.get_logger().info('Nav2 Goal Accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_goal_done_callback) 
    
    def nav2_goal_done_callback(self, future):
        result = future.result()

        if len(result.result.path.poses) == 0:
            self.get_logger().info('Nav2 Path contains no poses: Creating straight path')
            self.nav2_path = self.create_straight_path()
            return

        self.nav2_path = result.result.path
    
    def navigate(self):

        if self.robot_pose is None or self.path is None:
            return

        if not self.nav2_path:
            if not self.nav2_goal:
                self.calc_nav2_path() 
            return        
        
        self.path_pub.publish(self.nav2_path)

        if (self.distance(self.robot_pose, self.path.poses[self.waypoints_completed].pose) 
                < self.get_parameter('goal_dist_tolerance').value):
            self.waypoints_completed += 1
            if self.waypoints_completed == len(self.path.poses):
                self.get_logger().info('COMPLETED GOAL')
                self.success_pub.publish(self.path.poses[self.waypoints_completed - 1])
                self.path = None
            self.nav2_path = None
            self.nav2_goal = None
            return
    
    def create_straight_path(self):
        goal = Waypoints.pose_deep_copy(self.path.poses[self.waypoints_completed].pose)

        if goal.position.x - self.robot_pose.position.x > 0:
            x_dir = 1
        else:
            x_dir = -1
        
        if goal.position.y - self.robot_pose.position.y > 0:
            y_dir = 1
        else:
            y_dir = -1

        path = Path()
        path.header.frame_id = 'map'

        if goal.position.x - self.robot_pose.position.x == 0.0:
            theta = .5 * math.pi
        else:
            theta = math.atan(abs(
                (goal.position.y - self.robot_pose.position.y) / (goal.position.x - self.robot_pose.position.x)
            ))

        curr_pose = Waypoints.pose_deep_copy(self.robot_pose)

        while (
            (abs(curr_pose.position.x - self.robot_pose.position.x) 
                < abs(goal.position.x - self.robot_pose.position.x)) and 
            (abs(curr_pose.position.y - self.robot_pose.position.y) 
                < abs(goal.position.y - self.robot_pose.position.y))
        ):
            path.poses.append(PoseStamped(pose=Waypoints.pose_deep_copy(curr_pose)))

            curr_pose.position.x += math.cos(theta) * 0.2 * x_dir
            curr_pose.position.y += math.sin(theta) * 0.2 * y_dir

        path.poses.append(PoseStamped(pose=goal))

        return path
    
    def pose_deep_copy(pose:Pose):
        copy = Pose()
        copy.position.x = pose.position.x
        copy.position.y = pose.position.y
        copy.position.z = pose.position.z
        copy.orientation.x = pose.orientation.x
        copy.orientation.y = pose.orientation.y
        copy.orientation.z = pose.orientation.z
        copy.orientation.w = pose.orientation.w

        return copy

def main(args=None):
    
    rclpy.init(args=args)

    node = Waypoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
