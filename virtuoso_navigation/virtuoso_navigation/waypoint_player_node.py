import rclpy
from rclpy.node import Node
import os
from pathlib import Path
import yaml
from nav_msgs.msg import Path as NavPath
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped

class WaypointPlayerNode(Node):

    def __init__(self):
        super().__init__('waypoint_player')

        self.declare_parameter('file_num', -1)

        self.ll_points = None
        self.orientations = None

        self.populate_waypoints()

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')

        self.path_pub = self.create_publisher(NavPath, '/navigation/set_path', 10)
    
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

        self.waypoint_num = 0

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
    
    def send_next_waypoint(self):

        if self.waypoint_num >= len(self.ll_points):
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
        self.req.ll_point = GeoPoseStamped()
        self.req.ll_point.pose.position.latitude = self.ll_points[self.waypoint_num][0]
        self.req.ll_point.pose.position.longitude = self.ll_points[self.waypoint_num][1]

        map_dest = self.fromLL_cli.call_async(self.req)
        map_dest.add_done_callback(ll_callback) 


def main(args=None):
    rclpy.init(args=args)

    node = WaypointPlayerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()