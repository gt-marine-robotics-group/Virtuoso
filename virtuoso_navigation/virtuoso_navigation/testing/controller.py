from argparse import Action
import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from rclpy.action import ActionClient

class Controller(Node):

    def __init__(self):
        super().__init__('test_controller')

        self.client = ActionClient(self, FollowPath, 'follow_path')

        self.client.wait_for_server()

        self.goal = FollowPath.Goal()
        self.goal.path.header.frame_id = 'odom'
        self.goal.path.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, 20):

            pose_stamped = PoseStamped()
            pose = Pose()
            pose.position.x = float(i)
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation = Quaternion()

            pose_stamped.pose = pose
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.goal.path.poses.append(pose_stamped)

        self.client.send_goal(self.goal)


def main(args=None):
    
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    