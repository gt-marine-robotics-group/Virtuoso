import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped

class StationKeepingNode(Node):

    def __init__(self):
        super().__init__('navigation_station_keeping')

        self.enable_sub = self.create_subscription(Empty, '/navigation/station_keep', 
            self.enable_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry',
            self.odom_callback, 10)
        
        self.path_pub = self.create_publisher(Path, '/navigation/plan', 10)
        
        self.odom = None
    
    def odom_callback(self, msg:Odometry):
        self.odom = msg
    
    def enable_callback(self, msg:Empty):
        # should have odom_callback on separate thread and then run 
        # while (self.odom is None) pass
        if not self.odom:
            return

        self.get_logger().info('Station Keeping Enabled')
        
        path = Path()
        path.poses.append(PoseStamped(pose=self.odom.pose.pose))

        self.path_pub.publish(path)
        

def main(args=None):
    
    rclpy.init(args=args)

    node = StationKeepingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
