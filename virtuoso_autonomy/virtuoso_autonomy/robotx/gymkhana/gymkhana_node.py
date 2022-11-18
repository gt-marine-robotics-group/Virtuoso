import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from nav_msgs.msg import Odometry
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point32_to_pose_stamped
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from rclpy.time import Time
import tf_transformations

class Gymkhana(Node):

    def __init__(self):
        super().__init__('autonomy_gymkhana')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', 
            self.update_buoys, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.update_robot_pose, 10)

        self.robot_pose = None

        self.station_keeping_enabled = False
        self.station_keeping_complete = False

        self.channel_nav = ChannelNavigation()
        self.buoys = BoundingBoxArray()

    def update_robot_pose(self, msg:Odometry):
        ps = PoseStamped()
        ps.pose = msg.pose.pose
        self.robot_pose = ps
        if not self.station_keeping_enabled:
            self.enable_station_keeping()

    def enable_station_keeping(self):
        path = Path()
        path.poses.append(self.robot_pose)
        self.path_pub.publish(path)
        self.station_keeping_enabled = True
        self.get_logger().info('Station Keeping Enabled')
    
    def update_buoys(self, msg:BoundingBoxArray):
        self.buoys = msg
        if (not self.channel_nav.curr_channel):
            self.nav_to_next_midpoint()
    
    def nav_to_next_midpoint(self):
        if self.channel_nav.end_nav:
            return

        if self.robot_pose is None:
            return
        
        if not self.station_keeping_complete:
            return

        # self.get_logger().info(str(list(map(lambda b: b.value, self.buoys.boxes))))
        buoyPoses = list(point32_to_pose_stamped(b.centroid) for b in self.buoys.boxes if b.value >= 1)
        # self.get_logger().info(str(len(buoyPoses)))
        channel = self.channel_nav.find_channel(buoyPoses, self.robot_pose)
        if channel is None:
            return

        mid = ChannelNavigation.find_midpoint(channel[0], channel[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.path_pub.publish(path)

    def nav_success(self, msg:PoseStamped):
        # 1 less than number of channels needed to navigate
        # For gymkhana, this number will be 5
        if len(self.channel_nav.channels) == 2:
            self.channel_nav.end_nav = True
        
        if not self.station_keeping_complete:
            self.station_keeping_complete = True

        self.nav_to_next_midpoint()


def main(args=None):
    
    rclpy.init(args=args)

    node = Gymkhana()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()