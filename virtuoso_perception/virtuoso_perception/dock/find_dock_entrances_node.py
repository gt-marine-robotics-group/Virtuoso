import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8

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

        self.create_timer(1.0, self.send_ready)
    
    def points_callback(self, msg):
        self.points = msg
    
    def start_callback(self, msg):
        self.search_requested = True
    
    def send_ready(self):
        if self.search_requested:
            return
        self.ready_pub.publish(Int8(data=1))


def main(args=None):
    rclpy.init(args=args)
    node = FindDockEntrancesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()