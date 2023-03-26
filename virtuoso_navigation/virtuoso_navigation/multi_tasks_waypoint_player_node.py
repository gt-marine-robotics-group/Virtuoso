import rclpy
from rclpy.node import Node
from pathlib import Path
from nav_msgs.msg import Path as NavPath
import os
import yaml
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint

class MultiTasksWaypointPlayerNode(Node):

    def __init__(self):
        super().__init__('navigation_multi_tasks_waypoint_player')

        self.declare_parameter('file_num', -1)

        self.ll_points = None
        self.orientations = None

        self.populate_waypoints()
        self.get_logger().info(str(self.ll_points))

        self.waypoint_num = 0

        self.nav_success_sub = self.create_subscription(PoseStamped,
            '/navigation/success', self.nav_success_callback ,10)

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
        pass


def main(args=None):
    rclpy.init(args=args)

    node = MultiTasksWaypointPlayerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()