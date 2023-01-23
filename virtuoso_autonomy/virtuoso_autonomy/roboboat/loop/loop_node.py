import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Empty

class LoopNode(Node):

    def __init__(self):
        super().__init__('autonomy_loop')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.station_keeping_pub = self.create_publisher(Empty, 
            '/navigation/station_keep', 10)


def main(args=None):
    rclpy.init(args=args)

    node = LoopNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()