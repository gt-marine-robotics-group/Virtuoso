import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import CostmapPoints
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from .utils.astar import astar 
from .utils.get_nodes import get_nodes
from .utils.create_path import create_path

class Astar(Node):

    def __init__(self):
        super().__init__('astar')

        self.points_sub = self.create_subscription(CostmapPoints, '/path_finding/points', self.update_points, 10)
        self.path_pub = self.create_publisher(Path, '/path_finding/path', 10)

        self.goal_sub = self.create_subscription(Point, '/path_finding/goal', self.run_astar, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.set_pos, 10)

        self.map = CostmapPoints() 
        self.pos = None

    def update_points(self, msg:CostmapPoints):
        self.map = msg

    def set_pos(self, msg:Odometry):
        self.pos = msg.pose.pose.position

    def run_astar(self, msg:Point):
        if(self.pos is None):
            self.pos = Point()
            self.pos.x = 0.0
            self.pos.y = 0.0
            self.pos.z = 0.0

        if len(self.map.data) == 0: return

        all_nodes = get_nodes(self.pos, msg, self.map.data) 
        nodes = all_nodes[2]
        start_node = all_nodes[0]
        end_node = all_nodes[1]

        width = self.map.width
        height = self.map.height
        
        positions = astar(nodes, start_node, end_node, (width, height))
        
        path = create_path(positions)

        self.path_pub.publish(path)

    
def main(args=None):
    
    rclpy.init(args=args)

    node = Astar()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()