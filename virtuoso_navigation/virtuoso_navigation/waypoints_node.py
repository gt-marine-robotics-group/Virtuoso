from math import sqrt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import tf_transformations
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from virtuoso_navigation.planners.Planner import Planner
from virtuoso_navigation.planners.StraightPath import StraightPath
from virtuoso_navigation.planners.RRT import RRT

class Waypoints(Node):

    def __init__(self):
        super().__init__('navigation_waypoints')

        self.waypoints_sub = self.create_subscription(Path, '/navigation/set_waypoints', self.set_waypoints, 10)
        self.translate_sub = self.create_subscription(Path, '/navigation/set_trans_waypoints', 
            self.set_trans_waypoints, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.odom_callback, 10)

        self.map_sub = self.create_subscription(OccupancyGrid, '/mapping/occupancy_map',
            self.map_callback, 10)

        self.success_pub = self.create_publisher(PoseStamped, '/navigation/success', 10)
        
        self.path_pub = self.create_publisher(Path, '/navigation/plan', 10)
        self.is_trans_pub = self.create_publisher(Bool, '/controller/is_translation', 10)

        self.declare_parameters(namespace='', parameters=[
            ('debug', False),
            ('only_translate', False),
            ('goal_dist_tolerance', 0.0),
            ('goal_rotation_tolerance', 0.0),
            ('planner', ''),
            ('inflation_layer', 0.0),
            ('rrt.step_dist', 0.0),
            ('rrt.line_collision_check_granularity', 0.0),
            ('rrt.debug_iteration_time', 0.0)
        ])

        self.waypoints_completed = 0
        self.waypoints = None
        self.waypoint_yaws = None
        self.path = None

        planner_chosen = self.get_parameter('planner').value
        inflation_layer = self.get_parameter('inflation_layer').value
        if planner_chosen == 'STRAIGHT':
            self.planner: Planner = StraightPath(inflation_layer)
        elif planner_chosen == 'RRT':
            self.planner: Planner = RRT(
                inflation_layer,
                self.get_parameter('rrt.step_dist').value,
                self.get_parameter('rrt.line_collision_check_granularity').value,
                self.get_parameter('rrt.debug_iteration_time').value
            )
        else:
            raise 'No valid planner chosen.'

        if self.get_parameter('debug').value:
            self.planner.node = self

            if planner_chosen == 'RRT':
                self.rrt_tree_pub = self.create_publisher(Marker, '/navigation/rrt_tree', 10)

        self.create_timer(.1, self.navigate)
    
    def map_callback(self, map: OccupancyGrid):
        self.planner.map = map

    def odom_callback(self, odom:Odometry):
        self.planner.robot_pose = odom.pose.pose

    def within_goal_tolerance(self, p1:Pose, p2:Pose):
        if (sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)
            > self.get_parameter('goal_dist_tolerance').value):
            return False
        
        rq = self.planner.robot_pose.orientation
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

    def set_trans_waypoints(self, msg:Path):
        self.set_waypoints(msg, True) 
    
    def calc_path(self):
        self.get_logger().info('Creating path')
        self.path = self.planner.create_path(
            Planner.pose_deep_copy(self.waypoints.poses[self.waypoints_completed].pose)
        )
        self.path.header.frame_id = 'map'

    def navigate(self):

        if self.planner.robot_pose is None or self.planner.map is None or self.waypoints is None:
            return

        if not self.path:
            if self.waypoints_completed < len(self.waypoints.poses):
                self.calc_path() 
            return        
        
        self.path_pub.publish(self.path)

        if self.within_goal_tolerance(self.planner.robot_pose, self.waypoints.poses[self.waypoints_completed].pose):
            self.waypoints_completed += 1
            if self.waypoints_completed == len(self.waypoints.poses):
                self.get_logger().info('COMPLETED GOAL')
                self.success_pub.publish(self.waypoints.poses[self.waypoints_completed - 1])
                self.waypoints = None
            self.path = None
            return

def main(args=None):
    
    rclpy.init(args=args)

    node = Waypoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
