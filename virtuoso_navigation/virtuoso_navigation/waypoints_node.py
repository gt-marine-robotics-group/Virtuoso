from math import sqrt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import tf_transformations

class Waypoints(Node):

    def __init__(self):
        super().__init__('navigation_waypoints')

        self.waypoints_sub = self.create_subscription(Path, '/navigation/set_path', self.set_waypoints, 10)
        self.translate_sub = self.create_subscription(Path, '/navigation/set_trans_path', 
            self.set_trans_path, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)

        self.success_pub = self.create_publisher(PoseStamped, '/navigation/success', 10)
        
        self.path_pub = self.create_publisher(Path, '/navigation/plan', 10)
        self.is_trans_pub = self.create_publisher(Bool, '/controller/is_translation', 10)

        self.declare_parameters(namespace='', parameters=[
            ('only_translate', False),
            ('goal_dist_tolerance', 0.0),
            ('goal_rotation_tolerance', 0.0)
        ])

        self.waypoints_completed = 0
        self.waypoints = None
        self.waypoint_yaws = None
        self.path = None
        self.robot_pose = None

        self.create_timer(.1, self.navigate)
    
    def odom_callback(self, odom:Odometry):
        self.robot_pose = odom.pose.pose

    def within_goal_tolerance(self, p1:Pose, p2:Pose):
        if (sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)
            > self.get_parameter('goal_dist_tolerance').value):
            return False
        
        rq = self.robot_pose.orientation
        euler = tf_transformations.euler_from_quaternion([
            rq.x, rq.y, rq.z, rq.w
        ])

        if (abs(self.waypoint_yaws[self.waypoints_completed] - euler[2])
            > self.get_parameter('goal_rotation_tolerance').value):
            return False

        return True    
    
    def set_waypoints(self, msg:Path, is_trans=False):

        self.get_logger().info('Setting Waypoints')

        self.waypoints = msg
        self.waypoints_completed = 0

        self.waypoint_yaws = list()
        for pose in self.waypoints.poses:
            rq = pose.pose.orientation
            euler = tf_transformations.euler_from_quaternion([
                rq.x, rq.y, rq.z, rq.w
            ])
            self.waypoint_yaws.append(euler[2])

        self.path = None

        self.is_trans_pub.publish(
            Bool(data= is_trans or self.get_parameter('only_translate').value)
        )

    def set_trans_path(self, msg:Path):
        self.set_waypoints(msg, True) 
    
    def calc_path(self):
        self.get_logger().info('Creating Straight path')
        self.path = self.create_straight_path()

    def navigate(self):

        if self.robot_pose is None or self.waypoints is None:
            return

        if not self.path:
            if self.waypoints_completed < len(self.waypoints.poses):
                self.calc_path() 
            return        
        
        self.path_pub.publish(self.path)

        if self.within_goal_tolerance(self.robot_pose, self.waypoints.poses[self.waypoints_completed].pose):
            self.waypoints_completed += 1
            if self.waypoints_completed == len(self.waypoints.poses):
                self.get_logger().info('COMPLETED GOAL')
                self.success_pub.publish(self.waypoints.poses[self.waypoints_completed - 1])
                self.waypoints = None
            self.path = None
            return
    
    def create_straight_path(self):
        goal = Waypoints.pose_deep_copy(self.waypoints.poses[self.waypoints_completed].pose)

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
