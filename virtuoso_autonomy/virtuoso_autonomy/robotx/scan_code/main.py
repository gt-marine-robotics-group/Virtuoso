import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray

class ScanCode(Node):

    def __init__(self):
        super().__init__('autonomy_scan_code')

        self.scan_req_pub = self.create_publisher(Int8, '/perception/get_code', 10)

        self.scan_res_sub = self.create_subscription(Int32MultiArray, '/perception/code',
            self.code_callback, 10)

        self.req_sent = False

        self.create_timer(1.0, self.send_req)
    
    def send_req(self):
        if self.req_sent:
            return
        self.get_logger().info('Sending Scan Code Request')
        self.req_sent = True
        self.scan_req_pub.publish(Int8(data=1))
    
    def code_callback(self, msg):
        self.get_logger().info('Autonomy Received Code')

def main(args=None):
    rclpy.init(args=args)
    node = ScanCode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()