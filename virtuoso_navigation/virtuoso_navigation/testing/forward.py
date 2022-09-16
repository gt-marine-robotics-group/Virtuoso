import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path

class TestForward(Node):

    def __init__(self):
        super().__init__('test_set_path')

        self.pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        path = Path()
        pose_stamped = PoseStamped()

        pose = Pose()
        pose.position.x = 10

        pose_stamped.pose = pose
        pose_stamped.header.frame_id = 'wamv/base_link'

        path.poses.append(pose_stamped)

        self.pub.publish(path)


def main(args=None):
    
    rclpy.init(args=args)

    node = TestForward()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()