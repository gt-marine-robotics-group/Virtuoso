#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PointStamped, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np

class OccupancyMapGenerator(Node):
    def __init__(self) -> None:
        super().__init__('occupancy_map_generator')
        self.initialize_parameters()
        self.setup_subscribers_and_publishers()
        self.initialize_occupancy_grid()
        self.setup_transform_listener()

        self.map_origin_pub_debug.publish(self.map_origin)
        self.get_logger().info("Mapping node has started...")

    def initialize_parameters(self) -> None:
        parameters = [
            ('resolution', 1.0),
            ('grid_width', 100),
            ('grid_height', 100),
            ('update_delay', 0.5),
            ('point_weight', 13),
            ('decay_rate', 15),
        ]
        self.declare_parameters(namespace='', parameters=parameters)

        self.grid_res: float = self.get_parameter('resolution').value
        self.grid_width: int = self.get_parameter('grid_width').value
        self.grid_height: int = self.get_parameter('grid_height').value
        self.map_origin: Pose = self.generate_map_origin()
        self.point_weight: int = self.get_parameter('point_weight').value
        self.decay_rate: int = self.get_parameter('decay_rate').value

        self.latest_pointcloud: np.array = None

    def setup_subscribers_and_publishers(self) -> None:
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/mapping/voxels_in_map_frame', self.pointcloud_callback, 10)
        self.occupancy_map_pub = self.create_publisher(
            OccupancyGrid, '/mapping/occupancy_map', 10)
        self.update_occupancy_map_timer = self.create_timer(
            self.get_parameter('update_delay').value, self.update_occupancy_map)

        self.map_origin_pub_debug = self.create_publisher(
            Pose, '/mapping/map_origin', 10)

    def initialize_occupancy_grid(self) -> None:
        now = self.get_clock().now()
        self.occupancy_map = OccupancyGrid(
            header=Header(
                stamp=Time(sec=int(now.seconds_nanoseconds()[0]), 
                           nanosec=int(now.seconds_nanoseconds()[1])),
                frame_id='map'
            ),
            info=MapMetaData(
                resolution=self.grid_res,
                width=self.grid_width,
                height=self.grid_height,
                origin=self.map_origin
            ),
            data=[0] * (self.grid_width * self.grid_height)
        )

    def setup_transform_listener(self) -> None:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def generate_map_origin(self) -> Pose:
        half_width = (self.grid_width * self.grid_res) / 2
        half_height = (self.grid_height * self.grid_res) / 2

        pose = Pose()
        
        pose.position.x = -half_width
        pose.position.y = -half_height
        pose.position.z = 0.0

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose
    
    def pointcloud_callback(self, msg) -> None:
        points = np.array([[i[0], -i[1]] for i in point_cloud2.read_points(
            msg, field_names=('x', 'y'), skip_nans=True)])
        self.latest_pointcloud = points

    def update_occupancy_map(self) -> None:
        if self.latest_pointcloud is not None and self.latest_pointcloud.size != 0:
            self.process_pointcloud_data()
        else:
            self.get_logger().info('Problems receiving PointCloud')

        self.occupancy_map.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_map_pub.publish(self.occupancy_map)
        
        self.map_origin_pub_debug.publish(self.map_origin)

    def process_pointcloud_data(self) -> None:
        binned_cells = self.bin_points(self.latest_pointcloud, self.grid_res)
        curr_scan = np.zeros(self.grid_width * self.grid_height, dtype=np.int8)
        self.update_scan_data(curr_scan, binned_cells)

    def update_scan_data(self, curr_scan, binned_cells) -> None:
        for cell in binned_cells:
            cell_index = self.grid_width * (cell[1] - 1) + cell[0]
            curr_scan[cell_index] += 1

        for i, curr_val in enumerate(self.occupancy_map.data):
            self.occupancy_map.data[i] = self.calculate_new_cell_value(curr_scan[i], curr_val)

    def calculate_new_cell_value(self, cell_scan_count, current_value) -> int:
        if cell_scan_count > 0:
            return min(current_value + cell_scan_count * self.point_weight, 100)
        return max(current_value - self.decay_rate, 0)

    def bin_points(self, points, resolution) -> np.array:
        binned_points = np.floor(points.copy() / resolution)
        binned_points[:, 0] += self.grid_width // 2
        binned_points[:, 1] = self.grid_height // 2 - binned_points[:, 1]

        binned_points = binned_points[np.all((binned_points[:, :2] >= [0, 0]) & (
            binned_points[:, :2] < [self.grid_width, self.grid_height]), axis=1)]

        return binned_points.astype(np.int32)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
