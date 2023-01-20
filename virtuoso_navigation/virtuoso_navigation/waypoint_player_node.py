import rclpy
from rclpy.node import Node
import os
from pathlib import Path
import yaml

class WaypointPlayerNode(Node):

    def __init__(self):
        super().__init__('waypoint_player')

        self.declare_parameter('file_num', -1)

        self.ll_points = None
        self.orientations = None

        self.populate_waypoints()
    
    def populate_waypoints(self):
        file_num = self.get_parameter('file_num').value

        if file_num == -1:
            file = WaypointPlayerNode.find_recent_file()
        else:
            file = f'{Path.home()}/mrg/waypoints_raw/points_{file_num}.yaml'
        
        contents = None
        with open(file, 'r') as stream:
            contents = yaml.safe_load(stream)
        
        contents = contents['/**']['ros__parameters']

        self.ll_points = contents['ll_points']
        self.orientations = contents['orientations']

    def find_recent_file():

        all_files = os.listdir(f'{Path.home()}/mrg/waypoints_raw')

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

        return f'{Path.home()}/mrg/waypoints_raw/points_{max_num}.yaml'


def main(args=None):
    rclpy.init(args=args)

    node = WaypointPlayerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()