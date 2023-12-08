import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class TestRvizWaypointNode(Node):

    def __init__(self):
        super().__init__('test_rviz_waypoint')

        self.path_pub = self.create_publisher(Path, '/navigation/set_waypoints', 10)

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', 
            self.goal_callback, 10)
    
    def goal_callback(self, goal: PoseStamped):
        self.get_logger().info('Publishing waypoint')
        path = Path()
        path.poses.append(goal)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)

    node = TestRvizWaypointNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()