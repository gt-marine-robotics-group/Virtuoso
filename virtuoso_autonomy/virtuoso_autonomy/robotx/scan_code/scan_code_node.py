import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray, Empty
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from .scan_code_states import State

class ScanCode(Node):

    def __init__(self):
        super().__init__('autonomy_scan_code')

        self.scan_req_pub = self.create_publisher(Int8, '/perception/get_code', 10)
        self.station_keeping_pub = self.create_publisher(Empty, 
            '/navigation/station_keep', 10)

        self.scan_res_sub = self.create_subscription(Int32MultiArray, '/perception/code',
            self.code_callback, 10)
        
        self.state = State.START

        self.ready = False

        self.create_timer(1.0, self.execute)

    def execute(self):
        self.get_logger().info(str(self.state))
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            self.state = State.SENDING_SCAN_REQUEST
            return
        if self.state == State.SENDING_SCAN_REQUEST:
            self.send_req()
            return
        if self.state == State.SCANNING:
            return
    
    def enable_station_keeping(self):
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())
    
    def send_req(self):
        self.get_logger().info('Sending Scan Code Request')
        self.state = State.SCANNING
        self.scan_req_pub.publish(Int8(data=1))
    
    def code_callback(self, msg):
        self.get_logger().info('Autonomy Received Code')
        self.get_logger().info(str(msg))
        self.state = State.COMPLETE

def main(args=None):
    rclpy.init(args=args)
    node = ScanCode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()