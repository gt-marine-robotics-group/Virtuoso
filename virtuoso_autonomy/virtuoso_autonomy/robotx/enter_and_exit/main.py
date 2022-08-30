import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry
from ...utils.multi_gates import MultiGates
import random
import tf_transformations

class EnterAndExit(Node):

    def __init__(self):
        super().__init__('enter_and_exit')

        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success', self.nav_success, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', self.update_buoys, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.update_robot_pose, 10)

        self.multi_gates = MultiGates.MultiGates()

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
        if (self.state == 'finding_looping_cone'):
            self.loop_around_cone()

    def nav_success(self, msg:PoseStamped):
        pass

    def point32ToPoseStamped(p:Point32):
        ps = PoseStamped()
        ps.pose.position.x = p.x
        ps.pose.position.y = p.y
        ps.pose.position.z = p.z
        return ps

    def midpoint(self, p1:PoseStamped, p2:PoseStamped):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = (p1.pose.position.x + p2.pose.position.x) / 2
        ps.pose.position.y = (p1.pose.position.y + p2.pose.position.y) / 2

        ang = math.atan2((p1.pose.position.y - p2.pose.position.y), (p1.pose.position.x - p2.pose.position.x))

        rq = self.robot_pose.pose.orientation
        robot_euler = tf_transformations.euler_from_quaternion([rq.x, rq.y, rq.z, rq.w])

        if ang > math.pi * 2:
            ang = ang % (math.pi * 2)

        if abs(ang - robot_euler[2]) > abs((ang + math.pi) - robot_euler[2]):
            ang += math.pi
        
        quat = tf_transformations.quaternion_from_euler(0, 0, ang - (math.pi / 2))
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        return ps

    def navigate_to_enterance(self):

        buoyPoses = list(EnterAndExit.point32ToPoseStamped(b.centroid) for b in self.buoys.boxes)
        
        gates = self.multi_gates.find_gates(buoyPoses, self.robot_pose)

        self.get_logger().info(str(gates))

        if gates is None:
            return

        self.state = 'entering'

        randGate = gates[random.randint(0, 2)]

        mid = self.midpoint(randGate[0], randGate[1])

        path = Path()
        path.poses.append(mid)

        self.path_pub.publish(path)

    def loop_around_cone(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    node = EnterAndExit()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()