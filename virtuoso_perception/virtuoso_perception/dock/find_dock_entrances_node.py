import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Int8
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from rclpy.time import Time
from .find_dock_entrances import FindDockEntrances
from virtuoso_processing.utils.pointcloud import read_points, create_cloud_xyz32
from virtuoso_perception.utils.geometry_msgs import do_transform_point

class FindDockEntrancesNode(Node):

    def __init__(self):
        super().__init__('perception_find_dock_entrances')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.points_sub = self.create_subscription(PointCloud2, '/perception/voxel_voxels', 
            self.points_callback, 10)
        self.start_sub = self.create_subscription(Int8, '/perception/start_find_docks',
            self.start_callback, 10)
        
        self.ready_pub = self.create_publisher(Int8, '/perception/find_dock_entrances/ready', 10)

        self.first_two_pub = self.create_publisher(PointCloud2,
            '/perception/debug/first_two_entrances', 10)
        self.possible_pub = self.create_publisher(PointCloud2, 
            '/perception/debug/possible_entrances', 10)
        self.curr_pub = self.create_publisher(PointCloud2, '/perception/debug/current_entrances', 10)

        self.docks_pub = self.create_publisher(PointCloud2, '/perception/dock_entrances', 10)
        self.dock_ahead_pub = self.create_publisher(PointCloud2, '/perception/dock_ahead_entrance', 10)

        self.declare_parameters(namespace='', parameters=[
            ('debug', False)
        ])

        self.debugs = {
            'first_two': self.publish_first_two,
            'possible': self.publish_possible,
            'current': self.publish_current
        }
        
        self.points = None
        self.search_requested = False

        self.find_docks = FindDockEntrances()
        if self.get_parameter('debug').value:
            self.find_docks.node = self

    def points_callback(self, msg):
        self.points = msg
        self.get_points()
        self.find()
    
    def start_callback(self, msg):
        self.search_requested = True
    
    def get_points(self):
        self.find_docks.points = list()

        trans = None
        try:
            trans = self.tf_buffer.lookup_transform('wamv/base_link', 'map', Time())
        except Exception as e:
            self.get_logger().info('Failed Transform')
            return
        
        for point in read_points(self.points):
            p = PointStamped(point=Point(x=point[0], y=point[1]))
            trans_point = do_transform_point(p, trans)
            self.find_docks.points.append((trans_point.point.x, trans_point.point.y))
    
    def get_cloud(self, points):
        msg = PointCloud2()
        msg.header.frame_id = 'wamv/base_link'
        return create_cloud_xyz32(msg.header, list((p[0], p[1], 0) for p in points))
    
    def publish_first_two(self, points):
        self.first_two_pub.publish(self.get_cloud(points))

    def publish_possible(self, points):
        self.possible_pub.publish(self.get_cloud(points))
    
    def publish_current(self, points):
        self.curr_pub.publish(self.get_cloud(points))
    
    def find(self):
        if not self.search_requested:
            return
        
        self.find_docks.find_entrances()

        entrances = self.find_docks.get_entrances()
        if entrances is not None:
            self.docks_pub.publish(self.get_cloud(entrances))
        
        ahead_entrance = self.find_docks.get_ahead_entrance()
        if ahead_entrance is not None:
            self.dock_ahead_pub.publish(self.get_cloud(ahead_entrance))


def main(args=None):
    rclpy.init(args=args)
    node = FindDockEntrancesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()