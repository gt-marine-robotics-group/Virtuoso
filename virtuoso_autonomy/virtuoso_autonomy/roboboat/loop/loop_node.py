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
        self.translate_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.station_keeping_pub = self.create_publisher(Empty, 
            '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.declare_parameters(namespace='', parameters=[
            ('gate_use_lidar', False),
            ('gate_use_camera', False),
            ('gate_max_buoy_dist', 0.0),
            ('gate_extra_forward_nav', 0.0),
            
            ('loop_use_lidar', False),
            ('loop_use_camera', False),
            ('loop_max_buoy_dist', 0.0),

            ('looping_radius', 0.0)
        ])
        
        self.state = State.START

        self.channel_cli = self.create_client(Channel, 'channel')
        self.channel_call = None

        self.robot_pose:PoseStamped = None

        self.check_count = 0
        self.prev_channel_buoys:List[PoseStamped] = list()
        self.channel_wait_count = 0

        self.gate_midpoint:PoseStamped = None

        self.create_timer(1.0, self.execute)
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = PoseStamped(pose=msg.pose.pose)
    
    def nav_success_callback(self, msg:PoseStamped):
        if self.state == State.NAVIGATING_TO_GATE_MIDPOINT:
            self.gate_midpoint = self.robot_pose
            self.nav_forward()
        elif self.state == State.EXTRA_FORWARD_NAV:
            time.sleep(5.0)
            self.state = State.CHECKING_FOR_LOOP_BUOY
        elif self.state == State.NAVIGATING_STRAIGHT:
            self.state = State.CHECKING_FOR_LOOP_BUOY
        elif self.state == State.LOOPING:
            self.state = State.COMPLETE
    
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
        if self.state == State.EXTRA_FORWARD_NAV:
            return
        if self.state == State.CHECKING_FOR_LOOP_BUOY:
            self.find_loop_buoy()
            return
        if self.state == State.LOOPING:
            return
    
    def enable_station_keeping(self):
        if self.robot_pose is None:
            return
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())
    
    def nav_forward(self):
        self.state = State.EXTRA_FORWARD_NAV
        self.translate_pub.publish(Point(x=self.get_parameter('gate_extra_forward_nav').value)) 
    
    def nav_to_gate_midpoint(self):
        if self.robot_pose is None:
            return
        if self.channel_call is not None:
            return
        
        req = Channel.Request()
        req.left_color = 'red'
        req.right_color = 'green'
        req.use_lidar = self.get_parameter('gate_use_lidar').value
        req.use_camera = self.get_parameter('gate_use_camera').value
        req.max_dist_from_usv = self.get_parameter('gate_max_buoy_dist').value

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

        self.prev_channel_buoys.append(channel[0])
        self.prev_channel_buoys.append(channel[1])

        mid = ChannelNavigation.find_midpoint(channel[0], channel[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.state = State.NAVIGATING_TO_GATE_MIDPOINT
        self.channel_call = None
        self.path_pub.publish(path)
    
    def find_loop_buoy(self):
        if self.robot_pose is None:
            return
        if self.channel_call is not None and self.channel_wait_count < 5:
            self.channel_wait_count += 1
            return

        self.check_count += 1
        self.channel_wait_count = 0

        req = Channel.Request()
        req.left_color = 'blue'
        req.right_color = 'blue'
        req.use_lidar = self.get_parameter('loop_use_lidar').value
        req.use_camera = self.get_parameter('loop_use_camera').value
        req.max_dist_from_usv = self.get_parameter('loop_max_buoy_dist').value

        self.get_logger().info('sending channel request')
        self.channel_call = self.channel_cli.call_async(req)
        self.channel_call.add_done_callback(self.loop_buoy_response)
    
    def loop_buoy_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'channel response: {result}')

        null_point = Point(x=0.0,y=0.0,z=0.0)

        self.channel_call = None

        left_ps = point_to_pose_stamped(result.left)
        right_ps = point_to_pose_stamped(result.right)
        
        self.get_logger().info(f'check count: {self.check_count}')
        self.get_logger().info(f'prev1: {self.is_prev_buoy(left_ps)}')
        self.get_logger().info(f'prev2: {self.is_prev_buoy(right_ps)}')
        if ((result.left == null_point or self.is_prev_buoy(left_ps)) 
            and (result.right == null_point or self.is_prev_buoy(right_ps))):
            self.get_logger().info('No loop buoy found')
            if self.check_count >= 10:
                self.nav_straight()
            return
        
        if result.left == null_point:
            pose = right_ps
        elif result.right == null_point:
            pose = left_ps
        elif self.is_prev_buoy(left_ps):
            pose = right_ps
        elif self.is_prev_buoy(right_ps):
            pose = left_ps
        elif (distance_pose_stamped(left_ps, self.robot_pose)
            < distance_pose_stamped(right_ps, self.robot_pose)):
            pose = left_ps
        else:
            pose = right_ps
        
        self.get_logger().info(f'pose: {pose}')
        
        path = LoopingBuoy.find_path_around_buoy(self.robot_pose, pose,
            looping_radius=self.get_parameter('looping_radius').value)
        
        self.gate_midpoint.pose.orientation = path.poses[-1].pose.orientation
        path.poses.append(self.gate_midpoint)

        self.state = State.LOOPING
        self.channel_call = None
        self.path_pub.publish(path)
    
    def is_prev_buoy(self, buoy:PoseStamped):
        for pose in self.prev_channel_buoys:
            if distance_pose_stamped(buoy, pose) < 3:
                return True
        return False
        
    def nav_straight(self):
        self.check_count = 0

        self.state = State.NAVIGATING_STRAIGHT
        self.translate_pub.publish(Point(x=10.0))

def main(args=None):
    rclpy.init(args=args)

    node = LoopNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
