import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from tf2_ros.buffer import Buffer
from rclpy.time import Time
from tf2_ros.transform_listener import TransformListener
from virtuoso_perception.utils.geometry_msgs import do_transform_pose_stamped

class SingleWaypointNode(Node):

    def __init__(self):
        super().__init__('navigation_single_waypoint')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoint_sub = self.create_subscription(Pose, '/navigation/set_single_waypoint', 
            self.waypoint_callback, 10)

        self.set_path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
    
    def waypoint_callback(self, msg:Pose):
        trans = None

        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
        except Exception as e:
            self.get_logger().info('FAILED TRANSFORM')
            self.get_logger().info(str(e))
            return
        
        path = Path()
        ps = PoseStamped(pose=msg)

        pose = do_transform_pose_stamped(ps, trans)
        path.poses.append(pose)

        self.set_path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)

    node = SingleWaypointNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()