import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry
from ...utils.multi_gates.multi_gates import MultiGates
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point32_to_pose_stamped
import random
import tf_transformations

class EnterAndExit(Node):

    def __init__(self):
        super().__init__('autonomy_enter_and_exit')

        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success', self.nav_success, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', self.update_buoys, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.update_robot_pose, 10)

        self.multi_gates = MultiGates()

        self.robot_pose = None
        self.buoys = BoundingBoxArray()
        self.state = 'finding_enterance' # other states are 'entering', 'finding_loop_cone', 'looping'

    def update_robot_pose(self, msg:Odometry):
        ps = PoseStamped()
        ps.pose = msg.pose.pose
        self.robot_pose = ps

    def update_buoys(self, msg:BoundingBoxArray):
        self.buoys = msg
        if (self.state == 'finding_enterance'):
            self.navigate_to_enterance()
        if (self.state == 'finding_loop_cone'):
            self.loop_around_cone()

    def nav_success(self, msg:PoseStamped):
        if self.state == 'entering':
            self.state = 'finding_loop_cone'

    def navigate_to_enterance(self):

        if self.robot_pose is None:
            return

        buoyPoses = list(point32_to_pose_stamped(b.centroid) for b in self.buoys.boxes)
        
        gates = self.multi_gates.find_gates(buoyPoses, self.robot_pose)

        self.get_logger().info(str(gates))

        if gates is None:
            return

        self.state = 'entering'

        randGate = gates[random.randint(0, 2)]

        mid = ChannelNavigation.find_midpoint(randGate[0], randGate[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.path_pub.publish(path)

    def loop_around_cone(self):

        if self.robot_pose is None:
            return
        
        buoyPoses = list(point32_to_pose_stamped(b.centroid) for b in self.buoys.boxes)

        looping_buoy = self.multi_gates.find_looping_buoy(buoyPoses, self.robot_pose)

        if looping_buoy is None:
            return

        # Just go to the looping buoy for now to test
        
        self.state = 'looping'
        # path = Path()
        # path.poses.append(looping_buoy)
        path = self.multi_gates.find_path_around_buoy(looping_buoy, self.robot_pose)
        self.path_pub.publish(path)

        pass

def main(args=None):
    rclpy.init(args=args)

    node = EnterAndExit()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()