import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2
from virtuoso_msgs.action import ApproachTarget
from virtuoso_perception.utils.pointcloud import read_points
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class ApproachTargetNode(Node):

    def __init__(self):
        super().__init__('navigation_approach_target')

        self.cb_group1 = MutuallyExclusiveCallbackGroup()
        self.cb_group2 = MutuallyExclusiveCallbackGroup()

        self.action_server = ActionServer(self, ApproachTarget, 'approach_target', 
            self.action_server_callback, callback_group=self.cb_group1)

        self.lidar_sub = self.create_subscription(PointCloud2, '/perception/lidar/points_shore_filtered', 
            self.lidar_callback, 10, callback_group=self.cb_group2)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10, callback_group=self.cb_group2)

        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10,
            callback_group=self.cb_group1)
        
        self.points:PointCloud2 = None
        self.translating = False

        self.timeout = 0
    
    def nav_success_callback(self, msg):
        self.get_logger().info('approach finished 1 trans')
        self.translating = False
    
    def lidar_callback(self, msg:PointCloud2):
        self.points = msg
    
    def action_server_callback(self, goal_handle):
        self.get_logger().info('Received action request')
        self.get_logger().info(f'handle: {goal_handle.request}')
        self.timeout = 0

        while self.points is None:
            self.timeout += 0.1
            self.get_logger().info('Waiting for lidar points')
            time.sleep(0.1)  
        
        while True:
            self.timeout += 0.1

            if self.timeout > goal_handle.request.timeout:
                goal_handle.succeed()
                result = ApproachTarget.Result()
                result.success = False
                return result

            if self.translating:
                time.sleep(0.1)
                continue

            dist = self.find_dist_to_target(goal_handle.request.scan_width)
            self.get_logger().info(f'dist: {dist}')

            if dist == -1:
                time.sleep(0.1)
                continue

            if dist > goal_handle.request.approach_dist + .1:
                self.translating = True 
                self.trans_pub.publish(Point(x=dist-goal_handle.request.approach_dist))
            else:
                break
        
        goal_handle.succeed()
        
        result = ApproachTarget.Result()
        result.success = True

        return result
    
    def find_dist_to_target(self, scan_width:float):
        # points = [] 
        x_sum = 0
        num_points = 0

        for i, point in enumerate(read_points(self.points)):
            # points.append([0, 0, 0])
            if abs(point[1]) <= scan_width / 2:
                x_sum += point[0]
                num_points += 1
                # points[i][0] = point[0] 
                # points[i][1] = point[1]
                # points[i][2] = point[2] 
        
        if num_points == 0:
            return -1

        return x_sum / num_points


def main(args=None):
    rclpy.init(args=args)

    node = ApproachTargetNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()
