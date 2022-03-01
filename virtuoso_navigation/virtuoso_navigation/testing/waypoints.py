import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path

class TestWaypoints(Node):

    def __init__(self):
        super().__init__('test_set_path')
        self.pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        path = Path()
        pose_stamped = PoseStamped()
        
        pose = Pose()
        pose.position.x = 5.0
        pose.position.y = 5.0
        pose.position.z = 0.0
        pose.orientation = Quaternion()

        pose_stamped.pose = pose
        path.poses.append(pose_stamped)


        pose_stamped = PoseStamped()
        
        pose = Pose()
        pose.position.x = 5.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation = Quaternion()

        pose_stamped.pose = pose
        path.poses.append(pose_stamped)

        self.pub.publish(path)


def main(args=None):
    
    rclpy.init(args=args)

    node = TestWaypoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()