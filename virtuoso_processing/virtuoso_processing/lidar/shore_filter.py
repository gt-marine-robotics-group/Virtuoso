import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import math
from ..utils.pointcloud import read_points, create_cloud_xyz32
from ..utils.shore import ShoreFilter
from robot_localization.srv import FromLL
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ShoreFilterer(Node):

    def __init__(self):
        super().__init__('shore_filter')

        self.lidar_sub = self.create_subscription(PointCloud2, '/points_nonground', self.callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/points_shore_filtered', 10)

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.shore_filter = ShoreFilter

    def callback(self, msg:PointCloud2):
        if (not self.shore_filter.vrx_shore):
            self.shore_filter.create_shore(self.fromLL_cli, self.tf_buffer)

        self.get_logger().info(str(self.shore_filter.vrx_shore))
        self.publisher.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    node = ShoreFilterer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()