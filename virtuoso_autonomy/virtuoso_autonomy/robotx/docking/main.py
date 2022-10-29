import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class DockingNode(Node):

    def __init__(self):
        super().__init__('autonomy_docking', namespace='autonomy')

        self.find_docks_req_pub = self.create_publisher(Int8, '/perception/start_find_docks', 10)
        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        self.find_docks_ready_sub = self.create_subscription(Int8, '/perception/find_docks/ready',
            self.find_docks_ready_callback, 10)
        
        self.odom:Odometry = None
        self.find_docks_ready = False
        self.station_keeping_enabled = False
        self.find_docks_req_sent = False

        self.create_timer(1.0, self.send_find_docks_req)
    
    def odom_callback(self, msg):
        self.odom = msg
        if not self.station_keeping_enabled:
            self.enable_station_keeping()
    
    def find_docks_ready_callback(self, msg):
        self.find_docks_ready = True
    
    def enable_station_keeping(self):
        path = Path()
        pose_stamped = PoseStamped()
        pose_stamped.pose = self.odom.pose.pose
        path.poses.append(pose_stamped)
        self.path_pub.publish(path)
        self.station_keeping_enabled = True
        self.get_logger().info('Station Keeping Enabled')
    
    def send_find_docks_req(self):
        if not self.station_keeping_enabled:
            return
        
        if not self.find_docks_ready:
            return
        
        if self.find_docks_req_sent:
            return
        
        self.get_logger().info('Sending Find Docks Request')
        self.find_docks_req_sent = True
        self.find_docks_req_pub.publish(Int8(data=1))


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()