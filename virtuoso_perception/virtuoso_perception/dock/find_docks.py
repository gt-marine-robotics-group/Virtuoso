import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Path

class FindDocks(Node):

    def __init__(self):
        super().__init__('find_docks')


def main(args=None):
    rclpy.init(args=args)
    node = FindDocks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()