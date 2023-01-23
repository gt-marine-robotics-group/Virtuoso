import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from .loop_states import State
from virtuoso_msgs.srv import Channel
from geometry_msgs.msg import PoseStamped

class LoopNode(Node):

    def __init__(self):
        super().__init__('autonomy_loop')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.station_keeping_pub = self.create_publisher(Empty, 
            '/navigation/station_keep', 10)
        
        self.state = State.START

        self.channel_cli = self.create_client(Channel, 'channel')
        self.channel_call = None

        self.robot_pose:PoseStamped = None


def main(args=None):
    rclpy.init(args=args)

    node = LoopNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()