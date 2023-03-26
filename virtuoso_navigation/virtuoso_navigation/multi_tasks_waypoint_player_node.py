import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from pathlib import Path
from nav_msgs.msg import Path as NavPath
import os
import yaml
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint
from virtuoso_msgs.action import TaskWaypointNav
import time

class MultiTasksWaypointPlayerNode(Node):

    def __init__(self):
        super().__init__('navigation_multi_tasks_waypoint_player')

        self.declare_parameter('file_num', -1)

        self.cb_group1 = MutuallyExclusiveCallbackGroup()
        self.cb_group2 = MutuallyExclusiveCallbackGroup()

        self.action_server = ActionServer(self, TaskWaypointNav, 'task_waypoint_nav',
            self.action_server_callback, callback_group=self.cb_group1)

        self.ll_points = None
        self.orientations = None

        self.populate_waypoints()

        self.waypoint_num = 0
        self.task_num = -1
        self.completed_nav = False

        self.nav_success_sub = self.create_subscription(PoseStamped,
            '/navigation/success', self.nav_success_callback , 10, callback_group=self.cb_group2)

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')

        self.path_pub = self.create_publisher(NavPath, '/navigation/set_path', 10)

    def populate_waypoints(self):
        file_num = self.get_parameter('file_num').value

        if file_num == -1:
            file = MultiTasksWaypointPlayerNode.find_recent_file()
        else:
            file = f'{Path.home()}/mrg/semis_waypoints/points_{file_num}.yaml'
        
        contents = None
        with open(file, 'r') as stream:
            contents = yaml.safe_load(stream)
        
        contents = contents['/**']['ros__parameters']

        self.ll_points = dict()
        self.orientations = dict()

        for key, value in contents.items():
            if key[:2] == 'll':
                self.ll_points[key[-1]] = value
            else:
                self.orientations[key[-1]] = value

    def find_recent_file():

        all_files = os.listdir(f'{Path.home()}/mrg/semis_waypoints')

        max_num = -1
        for name in all_files:
            name_parts = name.split('.')[0].split('_')
            if len(name_parts) < 2: continue
            num = name_parts[1]
            if not num.isnumeric(): continue
            num = int(num)
            if num > max_num:
                max_num = num
        
        if max_num == -1: return None

        return f'{Path.home()}/mrg/semis_waypoints/points_{max_num}.yaml'
    
    def nav_success_callback(self, msg:PoseStamped):
        if self.task_num == -1:
            return
        self.waypoint_num += 1
        self.nav_to_next_waypoint()
    
    def action_server_callback(self, goal_handle):
        self.get_logger().info('Received action request')

        self.waypoint_num = 0
        self.task_num = goal_handle.request.task_num

        self.nav_to_next_waypoint()

        while not self.completed_nav:
            time.sleep(1.0)        
        
        self.completed_nav = False
        self.task_num = -1

        goal_handle.succeed()
        
        result = TaskWaypointNav.Result()
        result.success = True
        return result
        
    def nav_to_next_waypoint(self):
        while not self.fromLL_cli.service_is_ready():
            self.get_logger().info('waiting for service')
            time.sleep(1.0)
        
        if self.waypoint_num >= len(self.ll_points[str(self.task_num)]):
            self.completed_nav = True
            return
        
        def ll_callback(future):
            point = future.result().map_point

            pose_stamped = PoseStamped()
            pose_stamped.pose.position = point

            orientation = self.orientations[self.waypoint_num]
            pose_stamped.pose.orientation.x = orientation[0]
            pose_stamped.pose.orientation.y = orientation[1]
            pose_stamped.pose.orientation.z = orientation[2]
            pose_stamped.pose.orientation.w = orientation[3]

            path = NavPath()
            path.poses.append(pose_stamped)

            self.path_pub.publish(path)
        
        self.req = FromLL.Request() 
        self.req.ll_point = GeoPoint()
        self.req.ll_point.latitude = self.ll_points[str(self.task_num)][self.waypoint_num][0]
        self.req.ll_point.longitude = self.ll_points[str(self.task_num)][self.waypoint_num][1]
        self.req.ll_point.altitude = self.ll_points[str(self.task_num)][self.waypoint_num][2]

        map_dest = self.fromLL_cli.call_async(self.req)
        map_dest.add_done_callback(ll_callback) 


def main(args=None):
    rclpy.init(args=args)

    node = MultiTasksWaypointPlayerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()

    rclpy.shutdown()