import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WaypointSaverNode(Node):

    def __init__(self):
        super().__init__('waypoint_saver')

        self.key_sub = self.create_subscription(String, '/glyphkey_pressed', 
            self.key_callback, 10)
        
        self.prev_key = ''
        self.curr_key = ''

        self.ll_points = list()
        self.orientations = list()

    def key_callback(self, msg:String):
        self.prev_key = self.curr_key
        self.curr_key = msg.data

        if self.prev_key != self.curr_key:
            return
        
        self.curr_key = ''
        
        if self.prev_key == '>':
            self.add_waypoint()
        elif self.prev_key == '<':
            self.remove_waypoint()
    
    def add_waypoint(self):
        self.get_logger().info('adding waypoint')
    
    def remove_waypoint(self):
        self.get_logger().info('removing waypoint')

def main(args=None):
    rclpy.init(args=args)

    node = WaypointSaverNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()