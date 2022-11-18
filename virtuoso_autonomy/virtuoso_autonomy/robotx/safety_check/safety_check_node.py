import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Point32
from nav_msgs.msg import Odometry
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point32_to_pose_stamped
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from rclpy.time import Time
import tf_transformations
from .safety_check_states import State

class SafetyCheck(Node):

    def __init__(self):
        super().__init__('autonomy_safety_check')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.station_keeping_pub = self.create_publisher(Empty, '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', 
            self.update_buoys, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.update_robot_pose, 10)
        
        self.state = State.START

        self.robot_pose = None

        self.station_keeping_enabled = False

        self.channel_nav = ChannelNavigation()
        self.buoys = BoundingBoxArray()

        self.create_timer(1.0, self.execute)

    def update_robot_pose(self, msg:Odometry):
        ps = PoseStamped()
        ps.pose = msg.pose.pose
        self.robot_pose = ps
    
    def enable_station_keeping(self):
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())
    
    def update_buoys(self, msg:BoundingBoxArray):
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
        if self.state == State.NAVIGATING:
            return
    
    def nav_to_next_midpoint(self):

        if self.robot_pose is None:
            return

        buoyPoses = list(point32_to_pose_stamped(b.centroid) for b in self.buoys.boxes)
        channel = self.channel_nav.find_channel(buoyPoses, self.robot_pose)
        if channel is None:
            return

        mid = ChannelNavigation.find_midpoint(channel[0], channel[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.state = State.NAVIGATING
        self.path_pub.publish(path)

    def nav_success(self, msg:PoseStamped):
        # 1 less than number of channels needed to navigate
        if len(self.channel_nav.channels) == 1:
            self.state = State.COMPLETE
        else:
            self.state = State.FINDING_NEXT_GATE


def main(args=None):
    
    rclpy.init(args=args)

    node = SafetyCheck()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()