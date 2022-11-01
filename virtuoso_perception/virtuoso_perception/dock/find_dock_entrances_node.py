import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8
from .find_dock_entrances import FindDockEntrances
from virtuoso_processing.utils.pointcloud import read_points, create_cloud_xyz32

class FindDockEntrancesNode(Node):

    def __init__(self):
        super().__init__('find_dock_entrances')

        self.points_sub = self.create_subscription(PointCloud2, '/processing/super_voxels', 
            self.points_callback, 10)
        self.start_sub = self.create_subscription(Int8, '/perception/start_find_docks',
            self.start_callback, 10)
        
        self.ready_pub = self.create_publisher(Int8, '/perception/find_dock_entrances/ready', 10)
        
        self.points = None
        self.search_requested = False

        self.find_docks = FindDockEntrances()

        self.create_timer(1.0, self.send_ready)
    
    def points_callback(self, msg):
        self.points = msg
        self.get_points()
        self.find()
    
    def start_callback(self, msg):
        self.search_requested = True
    
    def send_ready(self):
        if self.search_requested:
            return
        self.ready_pub.publish(Int8(data=1))
    
    def get_points(self):
        self.find_docks.points = list()
        for point in read_points(self.points):
            self.find_docks.points.append(point)
    
    def find(self):
        if not self.search_requested:
            return
        
        self.find_docks.find_entrances(self)


def main(args=None):
    rclpy.init(args=args)
    node = FindDockEntrancesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()