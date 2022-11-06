import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class ScanCode(Node):

    def __init__(self):
        super().__init__('autonomy_scan_code')

        self.scan_req_pub = self.create_publisher(Int8, '/perception/get_code', 10)
        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.scan_ready_sub = self.create_subscription(Int8, '/perception/scan_code/ready', 
            self.scan_ready, 10)
        self.scan_res_sub = self.create_subscription(Int32MultiArray, '/perception/code',
            self.code_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.req_sent = False
        self.station_keeping_enabled = False
        self.odom = None

        self.ready = False

        self.create_timer(1.0, self.send_req)

    def scan_ready(self, msg):
        self.ready = True
    
    def odom_callback(self, msg):
        if self.odom is not None:
            return
        self.odom = msg
        self.enable_station_keeping(msg)

    def enable_station_keeping(self, msg):
        path = Path()
        pose_stamped = PoseStamped()
        pose_stamped.pose = self.odom.pose.pose
        path.poses.append(pose_stamped)
        self.path_pub.publish(path)
        self.station_keeping_enabled = True
        self.get_logger().info('Station Keeping Enabled')
    
    def send_req(self):
        if not self.station_keeping_enabled:
            return

        if not self.ready:
            return

        if self.req_sent:
            return

        self.get_logger().info('Sending Scan Code Request')
        self.req_sent = True
        self.scan_req_pub.publish(Int8(data=1))
    
    def code_callback(self, msg):
        self.get_logger().info('Autonomy Received Code')
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = ScanCode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()