import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry

class EnterAndExit(Node):

    def __init__(self):
        super.__init__('enter_and_exit')

        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success', self.nav_success, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', self.update_buoys, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.update_robot_pose, 10)


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

    def navigate_to_enterance(self):
        pass

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