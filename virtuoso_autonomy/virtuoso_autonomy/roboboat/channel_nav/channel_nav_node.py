import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty
from virtuoso_msgs.msg import BuoyArray
from virtuoso_msgs.srv import Channel
from .channel_nav_states import State
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point_to_pose_stamped

class ChannelNavNode(Node):

    def __init__(self):
        super().__init__('autonomy_channel_nav')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.station_keeping_pub = self.create_publisher(Empty, '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        self.buoys_sub = self.create_subscription(BuoyArray, '/perception/stereo/buoys', 
            self.buoys_callback, 10)
        
        self.state = State.START
        self.channel_nav = ChannelNavigation()

        self.channel_client = self.create_client(Channel, 'channel')
        self.channel_call = None
        
        self.robot_pose:PoseStamped = None
        self.buoys:BuoyArray = None

        self.create_timer(1.0, self.execute)

    def nav_success_callback(self, msg:PoseStamped):
        self.buoys = None
        if len(self.channel_nav.channels) == 2:
            self.state = State.COMPLETE
        else:
            self.state = State.FINDING_NEXT_GATE
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = PoseStamped(pose=msg.pose.pose)
    
    def buoys_callback(self, msg:BuoyArray):
        self.buoys = msg
    
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
    
    def enable_station_keeping(self):
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())
    
    def nav_to_next_midpoint(self):

        if self.robot_pose is None:
            return
        if self.buoys is None:
            return
        if self.channel_call is not None:
            return
        
        req = Channel.Request()
        req.left_color = 'red'
        req.right_color = 'green'
        req.max_dist_from_usv = 100.0

        self.channel_call = self.channel_client.call_async(req)
        self.channel_call.add_done_callback(self.channel_response)
    
    def channel_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'response: {result}')
        self.channel_call = None

        if (result.left == Point(x=0.0,y=0.0,z=0.0) or
            result.right == Point(x=0.0,y=0.0,z=0.0)):
            return

        buoy_poses = [
            point_to_pose_stamped(result.left),
            point_to_pose_stamped(result.right)
        ]

        channel = self.channel_nav.find_channel(buoy_poses, self.robot_pose)

        if channel is None:
            return

        mid = ChannelNavigation.find_midpoint(channel[0], channel[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.state = State.NAVIGATING
        self.path_pub.publish(path)


def main(args=None):
    
    rclpy.init(args=args)

    node = ChannelNavNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()