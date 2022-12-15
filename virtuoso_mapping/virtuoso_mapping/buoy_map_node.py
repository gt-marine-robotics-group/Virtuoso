import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import BuoyArray, Buoy
from geometry_msgs.msg import PointStamped, Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from virtuoso_perception.utils.geometry_msgs import do_transform_point, do_transform_pose
from virtuoso_processing.utils.pointcloud import create_cloud_xyz32
from collections import deque
from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from .mapped_buoy import MappedBuoy
from typing import List
import math
import tf_transformations

class BuoyMapNode(Node):

    cam_fov = 0.54
    frames = [
        "wamv/front_left_camera_link",
        "wamv/front_right_camera_link"
    ]

    def __init__(self):
        super().__init__('mapping_buoy_map')

        self.buoys_sub = self.create_subscription(BuoyArray, 
            '/perception/stereo/buoys', self.buoys_callback, 10)
        
        self.debug_pubs = {
            '/mapping/debug/filtered_buoy_points': self.create_publisher(PointCloud2, 
                '/mapping/debug/filtered_buoy_points', 10)
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_cam_to_map_transform:TransformStamped = None
        self.left_camera_pose:Pose = None
        self.right_camera_pose:Pose = None
        
        self.curr_buoys:BuoyArray = None

        self.mapped_buoys:List[MappedBuoy] = list()

    def find_camera_poses(self):
        right_to_map_trans:TransformStamped = self.tf_buffer.lookup_transform(
            'map',
            BuoyMapNode.frames[1],
            Time()
        )

        start = Pose(position=Point(x=0.0,y=0.0,z=0.0), 
            orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))
        
        self.left_camera_pose = do_transform_pose(start, self.left_cam_to_map_transform)
        self.right_camera_pose = do_transform_pose(start, right_to_map_trans)

    def buoys_callback(self, msg:BuoyArray):
        self.curr_buoys = msg
        self.execute()
    
    def transform_curr_buoys_to_map_frame(self):

        trans:TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                BuoyMapNode.frames[0],
                Time()
        )

        self.left_cam_to_map_transform = trans

        for buoy in self.curr_buoys.buoys:
            ps = PointStamped(point=buoy.location) 
            trans_ps = do_transform_point(ps, trans)
            buoy.location = trans_ps.point
    
    def debug_pcd(self, buoys:BuoyArray):
        pcd = PointCloud2()
        pcd.header.frame_id = 'map'
        pcd_points = list()
        for buoy in buoys.buoys:
            pcd_points.append([buoy.location.x, buoy.location.y, 0])
        
        pcd = create_cloud_xyz32(pcd.header, pcd_points)

        self.debug_pubs['/mapping/debug/filtered_buoy_points'].publish(pcd)
    
    def buoy_distance(buoy:Buoy):
        return math.sqrt(buoy.location.x**2 + buoy.location.y**2)
    
    def find_euler(quat:Quaternion):
        q = [quat.x, quat.y, quat.z, quat.w]

        euler = tf_transformations.euler_from_quaternion(q)

        return Vector3(x=euler[0], y=euler[1], z=euler[2])
    
    def filter_curr_buoys(self):
        index = 0
        while index < len(self.curr_buoys.buoys):
            if (BuoyMapNode.buoy_distance(self.curr_buoys.buoys[index]) > 15):
                self.curr_buoys.buoys.pop(index)
            else:
                index += 1
    
    def is_buoy_in_view(self, buoy:MappedBuoy, cam_location:Point, 
        cam_orientation:Vector3):

        cam_2d_vector = Vector3(y=math.sin(cam_orientation.z), 
            x=math.cos(cam_orientation.z))
        
        buoy_2d_vector = Vector3(x=buoy.location.x - cam_location.x,
            y=buoy.location.y - cam_location.y)
        
        c = cam_2d_vector
        b = buoy_2d_vector 
        angle = math.acos(((c.x * b.x) + (c.y * b.y)) / 
            (math.sqrt(c.x**2 + c.y**2) * math.sqrt(b.x**2 + b.y**2)))
        
        self.get_logger().info(f'angle: {angle * 180 / math.pi}') 

        return abs(angle) <= BuoyMapNode.cam_fov

    def mapped_buoys_in_view(self):
        left_cam_orientation = BuoyMapNode.find_euler(self.left_camera_pose.orientation)
        right_cam_orientation = BuoyMapNode.find_euler(self.right_camera_pose.orientation)

        for buoy in self.curr_buoys.buoys:
            self.is_buoy_in_view(buoy, self.left_camera_pose.position, 
                left_cam_orientation)
    
    def update_mapped_buoys(self):
        try:
            self.find_camera_poses()
        except Exception as e:
            self.get_logger().info('Camera pose transform failed')
            self.get_logger().info(str(e))
            return
        
        unmapped_indeces = set(i for i in range(len(self.curr_buoys.buoys)))

        self.mapped_buoys_in_view()

    def execute(self):

        self.filter_curr_buoys()

        camera_buoy_locs = self.curr_buoys.buoys.copy()

        try:
            self.transform_curr_buoys_to_map_frame()
        except Exception:
            self.get_logger().info('Camera to map transform failed')
            return

        self.debug_pcd(self.curr_buoys)

        self.update_mapped_buoys()


def main(args=None):
    
    rclpy.init(args=args)

    node = BuoyMapNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
