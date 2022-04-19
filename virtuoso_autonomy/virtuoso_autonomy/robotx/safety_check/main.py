import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from ...utils.channel_nav import ChannelNavigation
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from autoware_auto_perception_msgs.msg import BoundingBoxArray

class SafetyCheck(Node):

    def __init__(self):
        super().__init__('safety_check')

        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success', self.nav_success, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', self.update_buoys, 10)

        self.channel_nav = ChannelNavigation.ChannelNavigation()
        self.buoys = BoundingBoxArray()
    
    def update_buoys(self, msg:BoundingBoxArray):
        self.buoys = msg
        if (not self.channel_nav.curr_channel):
            self.nav_to_next_midpoint()

    def point32ToPoseStamped(p:Point32):
        ps = PoseStamped()
        ps.pose.position.x = p.x
        ps.pose.position.y = p.y
        ps.pose.position.z = p.z
        return ps
    
    def robotPose():
        ps = PoseStamped()
        ps.pose.position.x = 0.0
        ps.pose.position.y = 0.0
        ps.pose.position.z = 0.0
        return ps
    
    def midpoint(p1:PoseStamped, p2:PoseStamped):
        ps = PoseStamped()
        ps.pose.position.x = (p1.pose.position.x + p2.pose.position.x) / 2
        ps.pose.position.y = (p1.pose.position.y + p2.pose.position.y) / 2
        return ps
    
    def nav_to_next_midpoint(self):
        if self.channel_nav.end_nav:
            return

        buoyPoses = list(map(lambda b: SafetyCheck.point32ToPoseStamped(b.centroid), self.buoys.boxes))
        currPose =  SafetyCheck.robotPose()
        channel = self.channel_nav.find_channel(buoyPoses, currPose)
        if channel is None:
            return

        mid = SafetyCheck.midpoint(channel[0], channel[1])

        self.get_logger().info('ch 1 ' + str(channel[0].pose.position.x))
        self.get_logger().info('ch 2 ' + str(channel[1].pose.position.x))
        self.get_logger().info('mid ' + str(mid.pose.position.x))

        path = Path()
        path.poses.append(mid)

        self.path_pub.publish(path)

    def nav_success(self, msg:PoseStamped):
        # 1 less than number of channels needed to navigate
        if len(self.channel_nav.channels) == 0:
            self.channel_nav.end_nav = True

        self.nav_to_next_midpoint()


def main(args=None):
    
    rclpy.init(args=args)

    node = SafetyCheck()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()