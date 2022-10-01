import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from rclpy.time import Time
from tf2_ros.transform_listener import TransformListener

import numpy as np

class TestCircle(Node):

    def __init__(self):
        super().__init__('testing_circle')

        self.pub = self.create_publisher(Path, '/transformed_global_plan', 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.update_pose, 10)

        self.path_sent = False
        self.robot_pose = None

        self.declare_parameter('dist', 10.0)

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
        path.header.frame_id = 'map'

        p1 = PoseStamped()
        p1.pose = self.robot_pose.pose
        p1.header.frame_id = 'map'
        path.poses.append(p1)
        
        N = 6
        
        for i in range(1,N):
             p2 = PoseStamped()
             p2.pose.position.x = self.robot_pose.pose.position.x + (self.get_parameter('dist').value * np.sin(np.pi/N*i))
             p2.pose.position.y = self.robot_pose.pose.position.y + (self.get_parameter('dist').value * np.cos(np.pi/N*i))  - self.get_parameter('dist').value
             p2.pose.orientation = self.robot_pose.pose.orientation
             p2.header.frame_id = 'map'
             path.poses.append(p2)


        self.get_logger().info('PUBLISHING PATH')
        self.pub.publish(path)


def main(args=None):
    
    rclpy.init(args=args)

    node = TestCircle()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
