from rclpy.node import Node

class NodeHelper:

    def __init__(self, node:Node):

        self._node = node

    def _debug(self, msg:str):
        if self._node is None:
            return
        self._node.get_logger().info(msg)
    
    def _debug_pub(self, name:str, msg):
        if self._node is None:
            return
        self._node.debug_pub(name, msg)
    
    def _debug_pub_indexed(self, base:str, num:int, msg):
        if self._node is None:
            return
        self._node.debug_pub_indexed(base, num, msg)