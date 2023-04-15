#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from virtuoso_msgs.srv import DockCodesCameraPos

class TestFindDockCodesNode(Node):

    def __init__(self):
        super().__init__('test_find_dock_codes')

        self.declare_parameter('camera_base_topic', '')

        self.client = self.create_client(DockCodesCameraPos, f'find_dock_placard_offsets')
        self.req = None

        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        if self.req is not None:
            return
        
        self.client.wait_for_service(2.0)

        self.get_logger().info('sending req')

        msg = DockCodesCameraPos.Request()
        
        self.req = self.client.call_async(msg)
        self.req.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        self.req = None
        self.get_logger().info('received response')
        self.get_logger().info(str(future.result()))

def main(args=None):
    rclpy.init(args=args)

    node = TestFindDockCodesNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()