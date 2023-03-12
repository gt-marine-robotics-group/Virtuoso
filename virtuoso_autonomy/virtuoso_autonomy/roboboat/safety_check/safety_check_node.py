import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty
from virtuoso_msgs.srv import Channel, Rotate
from .safety_check_states import State
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point_to_pose_stamped
import time

class SafetyCheckNode(Node):

    def __init__(self):
        super().__init__('autonomy_safety_check')

        self.declare_parameters(namespace='', parameters=[
            ('gate_buoy_max_dist', 0.0),
            ('extra_forward_nav', 0.0),
            ('use_lidar', False),
            ('use_camera', False)
        ])

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.station_keeping_pub = self.create_publisher(Empty, '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        
        self.state = State.START
        self.channels_completed = 0

        self.channel_client = self.create_client(Channel, 'channel')
        self.channel_call = None
        
        self.robot_pose:PoseStamped = None

        self.create_timer(1.0, self.execute)

    def nav_success_callback(self, msg:PoseStamped):
        if self.channels_completed == 2:
            if self.state != State.COMPLETE:
                self.nav_forward()
            self.state = State.COMPLETE
        elif (self.state == State.EXTRA_FORWARD_NAV
            or self.get_parameter('extra_forward_nav').value == 0.0):
            time.sleep(5.0)
            self.state = State.FINDING_NEXT_GATE
        else:
            time.sleep(2.0)
            self.nav_forward()
        
    def nav_forward(self):
        self.state = State.EXTRA_FORWARD_NAV
        self.trans_pub.publish(Point(x=self.get_parameter('extra_forward_nav').value))
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = PoseStamped(pose=msg.pose.pose)
    
    def execute(self):
        self.get_logger().info(str(self.state))
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            self.state = State.FINDING_NEXT_GATE
            return
        if self.state == State.FINDING_NEXT_GATE:
            self.nav_to_next_midpoint()
            return
        if self.state == State.NAVIGATING:
            return
    
    def enable_station_keeping(self):
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())
    
    def nav_to_next_midpoint(self):

        if self.robot_pose is None:
            return
        if self.channel_call is not None:
            return
        
        req = Channel.Request()
        req.left_color = 'red'
        req.right_color = 'green'
        req.use_lidar = self.get_parameter('use_lidar').value
        req.use_camera = self.get_parameter('use_camera').value
        req.max_dist_from_usv = self.get_parameter('gate_buoy_max_dist').value

        self.channel_call = self.channel_client.call_async(req)
        self.channel_call.add_done_callback(self.channel_response)

    def rotate_response(self, future):
        result:Rotate.Response = future.result()
        self.get_logger().info(f'rotate response: {result}')
        time.sleep(10.0) 
        self.state = State.FINDING_NEXT_GATE
    
    def channel_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'response: {result}')

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

        self.state = State.NAVIGATING
        self.channel_call = None
        self.channels_completed += 1
        self.path_pub.publish(path)


def main(args=None):
    
    rclpy.init(args=args)

    node = SafetyCheckNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
