from rclpy.node import Node

class NodeHelper:

    def __init__(self, node:Node=None):

        self.node = node

    def _debug(self, msg:str):
        if self.node is None:
            return
        self.node.get_logger().info(msg)
    
    def _debug_pub(self, name:str, msg):
        if self.node is None:
            return
        self.node.debug_pub(name, msg)
    
    def _debug_pub_indexed(self, base:str, num:int, msg):
        if self.node is None:
            return
        self.node.debug_pub_indexed(base, num, msg)