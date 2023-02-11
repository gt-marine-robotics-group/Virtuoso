import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty
from .loop_states import State
from virtuoso_msgs.srv import Channel
from geometry_msgs.msg import PoseStamped, Point
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point_to_pose_stamped
from ...utils.math import distance_pose_stamped
from ...utils.looping_buoy.looping_buoy import LoopingBuoy
from typing import List
import time

class LoopNode(Node):

    def __init__(self):
        super().__init__('autonomy_loop')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.station_keeping_pub = self.create_publisher(Empty, 
            '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        
        self.state = State.START

        self.channel_cli = self.create_client(Channel, 'channel')
        self.channel_call = None

        self.robot_pose:PoseStamped = None

        self.check_count = 0
        self.prev_poses:List[PoseStamped] = list()

        self.create_timer(1.0, self.execute)
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = PoseStamped(pose=msg.pose.pose)
    
    def nav_success_callback(self, msg:PoseStamped):
        if self.state == State.NAVIGATING_TO_GATE_MIDPOINT:
            self.prev_poses.append(self.robot_pose)
            self.state = State.CHECKING_FOR_LOOP_BUOY
    
    def execute(self):
        self.get_logger().info(str(self.state))
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            self.state = State.FINDING_GATE
            return
        if self.state == State.FINDING_GATE:
            self.nav_to_gate_midpoint()
            return
        if self.state == State.NAVIGATING_TO_GATE_MIDPOINT:
            return
        if self.state == State.CHECKING_FOR_LOOP_BUOY:
            self.find_loop_buoy()
            return
        if self.state == State.LOOPING:
            return
    
    def enable_station_keeping(self):
        if self.robot_pose is None:
            return
        self.prev_poses.append(self.robot_pose)
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())
    
    def nav_to_gate_midpoint(self):
        if self.robot_pose is None:
            return
        if self.channel_call is not None:
            return
        
        req = Channel.Request()
        req.left_color = 'red'
        req.right_color = 'green'
        req.use_lidar = True # PARAM
        req.use_camera = True # PARAM
        req.max_dist_from_usv = 15.0 # PARAM

        self.channel_call = self.channel_cli.call_async(req)
        self.channel_call.add_done_callback(self.channel_response)
    
    def channel_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'channel response: {result}')

        null_point = Point(x=0.0,y=0.0,z=0.0)

        self.channel_call = None
        
        if result.left == null_point or result.right == null_point:
            self.get_logger().info('No Gate Found')
            return

        channel = (
            point_to_pose_stamped(result.left),
            point_to_pose_stamped(result.right)
        )

        mid = ChannelNavigation.find_midpoint(channel[0], channel[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.state = State.NAVIGATING_TO_GATE_MIDPOINT
        self.channel_call = None
        self.path_pub.publish(path)
    
    def find_loop_buoy(self):
        if self.robot_pose is None:
            return
        if self.channel_call is not None:
            return

        self.check_count += 1

        req = Channel.Request()
        req.left_color = 'blue'
        req.right_color = 'blue'
        req.use_lidar = True 
        req.max_dist_from_usv = 15.0 # PARAM

        self.channel_call = self.channel_cli.call_async(req)
        self.channel_call.add_done_callback(self.loop_buoy_response)
    
    def loop_buoy_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'channel response: {result}')

        null_point = Point(x=0.0,y=0.0,z=0.0)

        self.channel_call = None
        
        if result.left == null_point or result.right == null_point:
            self.get_logger().info('No loop buoy found')
            if self.check_count >= 3:
                self.nav_straight()
            return
        
        left_ps = point_to_pose_stamped(result.left)
        right_ps = point_to_pose_stamped(result.right)
        
        if result.left == null_point:
            pose = left_ps
        elif result.right == null_point:
            pose = right_ps
        elif (distance_pose_stamped(left_ps, self.robot_pose)
            < distance_pose_stamped(right_ps, self.robot_pose)):
            pose = left_ps
        else:
            pose = right_ps
        
        path = LoopingBuoy.find_path_around_buoy(self.robot_pose, pose)

        self.state = State.LOOPING
        self.channel_call = None
        self.path_pub.publish(path)
        
    def nav_straight(self):
        self.check_count = 0
        return

def main(args=None):
    rclpy.init(args=args)

    node = LoopNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()