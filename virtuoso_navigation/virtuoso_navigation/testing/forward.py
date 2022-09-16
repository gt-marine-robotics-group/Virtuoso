import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from rclpy.time import Time
from tf2_ros.transform_listener import TransformListener

class TestForward(Node):

    def __init__(self):
        super().__init__('test_set_path')

        self.pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.update_pose, 10)

        self.path_sent = False
        self.robot_pose = None

        self.create_timer(1.0, self.send_path)
    
    def update_pose(self, msg:Odometry):
        ps = PoseStamped()
        ps.pose = msg.pose.pose
        self.robot_pose = ps
    
    def send_path(self):

        if self.robot_pose is None or self.path_sent:
            return

        self.path_sent = True

        path = Path()

        self.robot_pose.pose.position.x += 10
        self.robot_pose.header.frame_id = 'map'

        path.poses.append(self.robot_pose)

        self.get_logger().info('PUBLISHING PATH')
        self.pub.publish(path)


def main(args=None):
    
    rclpy.init(args=args)

    node = TestForward()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()