import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point32_to_pose_stamped
from .enter_exit_states import State
import random
from .enter_exit import EnterExit

class EnterAndExitNode(Node):

    def __init__(self):
        super().__init__('autonomy_enter_and_exit')

        self.path_pub = self.create_publisher(Path, '/navigation/set_waypoints', 10)
        self.station_keeping_pub = self.create_publisher(Empty, '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', 
            self.update_buoys, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.update_robot_pose, 10)
        
        self.state = State.START

        self.robot_pose = None
        self.buoys = BoundingBoxArray()

        self.enter_exit = EnterExit()

        self.create_timer(1.0, self.execute)

    def execute(self):
        self.get_logger().info(str(self.state))
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            self.state = State.FINDING_ENTRANCE
            return
        if self.state == State.FINDING_ENTRANCE:
            self.navigate_to_enterance()
            return
        if self.state == State.NAVIGATING_TO_ENTRANCE:
            return
        if self.state == State.FINDING_LOOP_BUOY:
            self.loop_around_cone() 
            return
        if self.state == State.NAVIGATING_AROUND_BUOY:
            return

    def update_robot_pose(self, msg:Odometry):
        ps = PoseStamped()
        ps.pose = msg.pose.pose
        self.robot_pose = ps
        self.enter_exit.robot_pose = self.robot_pose
    
    def enable_station_keeping(self):
        self.station_keeping_pub.publish(Empty())
        self.state = State.STATION_KEEPING_ENABLED

    def update_buoys(self, msg:BoundingBoxArray):
        self.buoys = msg

    def nav_success(self, msg:PoseStamped):
        self.state = State(self.state.value + 1)

    def navigate_to_enterance(self):

        if self.robot_pose is None:
            return

        buoyPoses = list(point32_to_pose_stamped(b.centroid) for b in self.buoys.boxes)
        
        gates = self.enter_exit.find_gates(buoyPoses)

        if gates is None:
            return

        self.state = State.NAVIGATING_TO_ENTRANCE

        randGate = gates[random.randint(0, 2)]

        mid = ChannelNavigation.find_midpoint(randGate[0], randGate[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.path_pub.publish(path)

    def loop_around_cone(self):

        if self.robot_pose is None:
            return
        
        buoyPoses = list(point32_to_pose_stamped(b.centroid) for b in self.buoys.boxes)

        looping_buoy = self.enter_exit.find_looping_buoy(buoyPoses)

        if looping_buoy is None:
            return

        self.state = State.NAVIGATING_AROUND_BUOY

        path = self.enter_exit.find_path_around_buoy(looping_buoy)
        self.path_pub.publish(path)
    
    
def main(args=None):
    rclpy.init(args=args)

    node = EnterAndExitNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()