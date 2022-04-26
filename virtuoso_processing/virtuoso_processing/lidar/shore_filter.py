import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import math
from ..utils.pointcloud import read_points, create_cloud_xyz32
from ..utils.shore import ShoreFilter
from robot_localization.srv import FromLL
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.duration import Duration
from rclpy.time import Time

class ShoreFilterer(Node):

    def __init__(self):
        super().__init__('shore_filter')

        self.lidar_sub = self.create_subscription(PointCloud2, '/points_nonground', self.callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/points_shore_filtered', 10)

        self.poly_pub = self.create_publisher(MarkerArray, '/shore_polygon', 10)

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def callback(self, msg:PointCloud2):
        if (not self.fromLL_cli.service_is_ready()):
            return
        if (not ShoreFilter.vrx_shore):
            ShoreFilter.create_shore(self.fromLL_cli, self.tf_buffer)
            return

        filtered = ShoreFilter.filter_points(msg, self.tf_buffer)

        self.display_border_array()

        if filtered is not None:
            self.publisher.publish(filtered)
    
    # Mainly for debugging,
    # displays the points that make up the shore, labelled 1 - (1 - len)
    def display_border_array(self):
        markerArr = MarkerArray()
        for i, p in enumerate(ShoreFilter.vrx_border_mappoints):
            marker = Marker()
            marker.ns = 'polygon_border'
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.header.frame_id = 'map'
            marker.pose.position = p
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0).to_msg()
            marker.text = str(i)
            markerArr.markers.append(marker)

        self.poly_pub.publish(markerArr)


def main(args=None):
    
    rclpy.init(args=args)

    node = ShoreFilterer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()