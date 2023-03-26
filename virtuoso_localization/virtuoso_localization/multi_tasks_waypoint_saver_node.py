import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pathlib import Path
import os

class MultiTasksWaypointSaverNode(Node):

    def __init__(self):
        super().__init__('localization_multi_task_waypoint_saver')

        self.declare_parameters(namespace='', parameters=[
            ('num_tasks', 0)
        ])

        self.key_sub = self.create_subscription(String, '/glyphkey_pressed', 
            self.key_callback, 10)
        
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/filtered',
            self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.gps:NavSatFix = None
        self.odom:Odometry = None

        self.saved_to_yaml = False

        self.prev_key = ''
        self.curr_key = ''

        self.num_tasks = self.get_parameter('num_tasks').value

        self.curr_task = 1

        self.ll_points = dict()
        self.orientations = dict()
        for i in range(self.num_tasks):
            self.ll_points[str(i + 1)] = list()
            self.orientations[str(i + 1)] = list()
        self.get_logger().info(str(self.ll_points))

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
        elif self.prev_key == '+':
            self.inc_task()
        elif self.prev_key == '-':
            self.dec_task()
    
    def add_waypoint(self):
        if self.gps is None:
            self.get_logger().info('No GPS')
            return
        if self.odom is None:
            self.get_logger().info('No Odometry')
            return
        
        self.ll_points[str(self.curr_task)].append([self.gps.latitude, self.gps.longitude, 
            self.gps.altitude])
        self.orientations[str(self.curr_task)].append([
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        ])

        self.get_logger().info(f'Added Waypoint for task {self.curr_task}')
    
    def remove_waypoint(self):
        if len(self.ll_points[str(self.curr_task)]) == 0:
            self.get_logger().info(f'No Waypoints to remove for task {self.curr_task}')
            return
        
        self.ll_points[str(self.curr_task)].pop()
        self.orientations[str(self.curr_task)].pop()

        self.get_logger().info(f'Removed Waypoint for task {self.curr_task}')
    
    def inc_task(self):
        if self.curr_task == self.num_tasks:
            self.get_logger().info(f'Cannot increment: on final task ({self.curr_task})')
        else:
            self.get_logger().info(f'Incrementing to task {self.curr_task + 1}')
            self.curr_task += 1
    
    def dec_task(self):
        if self.curr_task == 1:
            self.get_logger().info('Cannot decrement: on first task')
        else:
            self.get_logger().info(f'Decrementing to task {self.curr_task - 1}')
            self.curr_task -= 1
    
    def save_to_yaml(self, path='/mrg/semis_waypoints'):
        self.get_logger().info('saving to yaml')

        all_files = os.listdir(f'{Path.home()}{path}')

        max_num = -1
        for name in all_files:
            name_parts = name.split('.')[0].split('_')
            if len(name_parts) < 2: continue
            num = name_parts[1]
            if not num.isnumeric(): continue
            num = int(num)
            if num > max_num:
                max_num = num
        
        with open(f'{Path.home()}{path}/points_{max_num + 1}.yaml', 'w') as file:
            file.writelines([
                '/**:\n',
                '  ros__parameters:\n',
                *(f'    ll_points_{i+1}: {self.ll_points[str(i+1)]}\n' for i in range(self.num_tasks)),
                *(f'    orientations_{i+1}: {self.orientations[str(i+1)]}\n' for i in range(self.num_tasks))
            ])

        self.get_logger().info('destroying node')
        self.destroy_node()
    
    def __del__(self):
        if self.saved_to_yaml:
            return
        self.get_logger().info('Saving to garbage YAML')


def main(args=None):
    rclpy.init(args=args)

    node = MultiTasksWaypointSaverNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()