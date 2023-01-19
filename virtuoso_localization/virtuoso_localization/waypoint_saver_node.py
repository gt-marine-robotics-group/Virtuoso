import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from typing import List
import os
from pathlib import Path

class WaypointSaverNode(Node):

    def __init__(self):
        super().__init__('waypoint_saver')

        self.key_sub = self.create_subscription(String, '/glyphkey_pressed', 
            self.key_callback, 10)
        
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/filtered',
            self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        
        self.prev_key = ''
        self.curr_key = ''

        self.gps:NavSatFix = None
        self.odom:Odometry = None

        self.ll_points:List[List[float]] = list() # [[lat, lon], ...]
        self.orientations:List[List[float]] = list() #[[x,y,z,w], ...]
    
    def gps_callback(self, msg:NavSatFix):
        self.gps = msg
    
    def odom_callback(self, msg:Odometry):
        self.odom = msg

    def key_callback(self, msg:String):
        self.prev_key = self.curr_key
        self.curr_key = msg.data

        if self.prev_key == '@' and self.curr_key == '!':
            self.save_to_yaml()
            return

        if self.prev_key != self.curr_key:
            return
        
        self.curr_key = ''
        
        if self.prev_key == '>':
            self.add_waypoint()
        elif self.prev_key == '<':
            self.remove_waypoint()
    
    def save_to_yaml(self):
        self.get_logger().info('saving to yaml')

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
        
        with open(f'{Path.home()}/mrg/waypoints_raw/points_{max_num + 1}.yaml', 'w') as file:
            file.writelines([
                '/**:\n',
                '\tros__parameters:\n',
                f'\t\tll_points: {self.ll_points}\n',
                f'\t\torientations: {self.orientations}\n'
            ])

        self.get_logger().info('destroying node')
        self.destroy_node()
    
    def add_waypoint(self):
        if self.gps is None:
            self.get_logger().info('No GPS')
            return
        if self.odom is None:
            self.get_logger().info('No Odometry')
            return
        
        self.ll_points.append([self.gps.latitude, self.gps.longitude]) 
        self.orientations.append([
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ])

        self.get_logger().info('Added Waypoint')
    
    def remove_waypoint(self):
        if len(self.ll_points) == 0:
            self.get_logger().info('No Waypoints to Remove')
            return
        
        self.ll_points.pop()
        self.orientations.pop()

        self.get_logger().info('Removed Waypoint')

def main(args=None):
    rclpy.init(args=args)

    node = WaypointSaverNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()