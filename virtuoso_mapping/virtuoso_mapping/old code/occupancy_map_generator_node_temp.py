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
from virtuoso_perception.utils.geometry_msgs import do_transform_point

import numpy as np


class OccupancyMapGenerator(Node):

    def __init__(self) -> None:
        # Initializing the Node
        super().__init__('occupancy_map_generator')

        # Initialize paramaters from YAML file
        self.declare_parameters(namespace='', parameters=[
            ('resolution', 1.0),
            ('grid_width', 100),
            ('grid_height', 100),
            ('update_delay', 0.5),
            ('point_weight', 13),
            ('decay_rate', 15),
        ])

        self.grid_res: float = self.get_parameter('resolution').value
        self.grid_width: int = self.get_parameter('grid_width').value
        self.grid_height: int = self.get_parameter('grid_height').value
        self.map_origin: Pose = self.generate_map_origin()

        # point_weight is the amount added to a grid per lidar point within the grid
        # (the more points the more confident the grid cell contains an obstacle)
        self.point_weight: int = self.get_parameter('point_weight').value

        # The rate at which a cell loses confidence if no points are detected within
        self.decay_rate: int = self.get_parameter('decay_rate').value

        # Publishers and Subscribers
        self.latest_pointcloud: np.array = None

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/perception/lidar/voxels',
            self.pointcloud_callback,
            10
        )

        self.occupancy_map_pub = self.create_publisher(
            OccupancyGrid,
            '/mapping/occupancy_map',
            10
        )

        # Initializing the OccupancyGrid map
        self.occupancy_map = OccupancyGrid(
            header=Header(
                stamp=Time(sec=int(self.get_clock().now().seconds_nanoseconds()[0]),
                           nanosec=int(self.get_clock().now().seconds_nanoseconds()[1])),
                frame_id='map'
            ),

            info=MapMetaData(
                resolution=self.grid_res,
                width=self.grid_width,
                height=self.grid_height,
                origin=self.map_origin
            )
        )
        
        # Setting map data to 0 (Representing empty space)
        self.occupancy_map.data = [0 for _ in range(self.grid_height * self.grid_width)]

        # Creating a Timer for how often to update the occupancy_map
        self.update_occupancy_map_timer = self.create_timer(
            self.get_parameter('update_delay').value, self.update_occupancy_map)

        self.get_logger().info("Mapping node has started...")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def generate_map_origin(self) -> Pose:
        delta_width_meters = (self.grid_width * self.grid_res) / 2
        delta_height_meters = (self.grid_height * self.grid_res) / 2

        pose = Pose()

        # Places the map_frame in the center of the gridmap
        pose.position.x = -delta_width_meters
        pose.position.y = -delta_height_meters
        pose.position.z = 0.0

        # Orientation is same as map_frame orientation
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        # self.get_logger().info(f'Pointcloud Frame: {msg.header.frame_id}')
        point_generator = point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)

        # points = list(point_generator)
        points = [[i[0], i[1]]  for i in point_generator]
        points_array: np.array = np.array(points)
        
        transformed_points = self.transform_points(points_array)
        
        self.latest_pointcloud = transformed_points
        # self.latest_pointcloud = points_array

    def transform_points(self, points: np.array) -> np.array:
        transformed_points = points.copy()
        # transformed_points[:, 1] *= -1
        
        trans = None
        try:
            trans = self.tf_buffer.lookup_transform('wamv/wamv/base_link', 'odom', Time())
        except Exception as e:
            self.get_logger().info('Occupancy Grid failed to find transform')
            return
        
        # self.get_logger().info(f'Transform: {trans}')

        for row in range(transformed_points.shape[0]):
            point = transformed_points[row]
            p = PointStamped(point=Point(x=float(point[1]), y=float(point[0])))
            # self.get_logger().info("")
            # self.get_logger().info(f'Before Transform: [{p.point.x}, {p.point.y}]')
            trans_point = do_transform_point(p, trans)
            # trans_point = p
            # self.get_logger().info(f'After Transform: [{trans_point.point.x}, {trans_point.point.x}]')
            transformed_points[row][0] = trans_point.point.x #- trans.transform.translation.x
            transformed_points[row][1] = trans_point.point.y #- trans.transform.translation.y

        return transformed_points
    
    def update_occupancy_map(self) -> None:
        
        if self.latest_pointcloud is not None and self.latest_pointcloud.size != 0:
            binned_cells: np.array = self.bin_points(self.latest_pointcloud,
                                                     self.grid_res)

            curr_scan = np.zeros((self.grid_width * self.grid_height), dtype=np.int8)

            for cell in binned_cells:
                cell_index = self.grid_width * cell[0] + cell[1]
                
                # Counting number of points in each cell
                curr_scan[cell_index] += 1
            
            for i in range(curr_scan.shape[0]):
                curr_val = self.occupancy_map.data[i]
                
                if curr_scan[i] > 0:
                    # Taking min to clamp within bounds of 100
                    self.occupancy_map.data[i] = min(
                        (curr_val + curr_scan[i] * self.point_weight), 100)
                else:
                    self.occupancy_map.data[i] = max(
                        curr_val - self.decay_rate, 0)
        else:
            self.get_logger().info('Problems receiveing PointCloud')

        now = self.get_clock().now()
        self.occupancy_map.header.stamp = now.to_msg()
        
        # self.get_logger().info(f'{self.occupancy_map}')
        self.occupancy_map_pub.publish(self.occupancy_map)
    
    def bin_points(self, points: np.array, resolution: float) -> np.array:
        
        binned_points = np.floor(points.copy() / resolution)

        binned_points[:, 0] = binned_points[:, 0] + self.grid_width // 2
        binned_points[:, 1] = self.grid_height // 2 - binned_points[:, 1]

        # Filtering the points to make sure they are in bounds and no repeat tiles
        binned_points = binned_points[np.all((binned_points[:, :2] >= [0, 0]) & (
            binned_points[:, :2] < [self.grid_width, self.grid_height]), axis=1)]

        # binned_points = np.unique(binned_points.astype(np.int32), axis=0)

        return binned_points.astype(np.int32)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapGenerator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
