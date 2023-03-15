import rclpy
from rclpy.node import Node
from .docking_states import State
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Point, Pose

class DockingNode(Node):

    def __init__(self):
        super().__init__('autonomy_docking')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.station_keeping_pub = self.create_publisher(Empty, '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.state = State.START

        self.robot_pose:Pose = None

        self.create_timer(1.0, self.execute)
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = msg.pose.pose
    
    def nav_success_callback(self):
        if self.state == State.APPROACHING_DOCK: self.state = State.ORIENTING
        elif self.state == State.SEARCH_TRANSLATING: self.state = State.SEARCHING_FOR_DOCK_CODE
        elif self.state == State.CENTER_TRANSLATING: self.state = State.CENTERING
        elif self.state == State.DOCKING: self.state = State.COUNTING_CODE
    
    def execute(self):
        self.get_logger().info(str(self.state))
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            # skip the next 2 states until implemented
            self.state = State.SEARCHING_FOR_DOCK_CODE
            return
        if self.state == State.APPROACHING_DOCK:
            return
        if self.state == State.ORIENTING:
            return
    
    def enable_station_keeping(self):
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)

    node = DockingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()