import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import CostmapPoints
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from .utils.astar import astar 
from .utils.get_nodes import get_nodes
from .utils.create_path import create_path
import math

class Astar(Node):

    def __init__(self):
        super().__init__('astar')

        self.points_sub = self.create_subscription(CostmapPoints, '/path_finding/points', self.update_points, 10)
        self.path_pub = self.create_publisher(Path, '/path_finding/path', 10)

        self.goal_sub = self.create_subscription(Point, '/path_finding/goal', self.run_astar, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', self.set_pos, 10)

        self.map = CostmapPoints() 
        self.pos = None 
        self.astar_pos = None
        self.goal = None
        self.no_path_found = False

    def update_points(self, msg:CostmapPoints):
        self.map = msg
        if self.no_path_found:
            self.run_astar(self.goal)

    def set_pos(self, msg:Odometry):
        self.pos = msg.pose.pose.position
        
        if self.goal is not None and math.sqrt((self.pos.x - self.astar_pos.x)**2 + (self.pos.y - self.pos.y)**2) > 1:
            self.run_astar(self.goal)

    def run_astar(self, msg:Point):
        if(self.pos is None):
            self.pos = Point()
            self.pos.x = 0.0
            self.pos.y = 0.0
            self.pos.z = 0.0

        if len(self.map.data) == 0:
            return

        self.goal = msg
        self.astar_pos = self.pos

        all_nodes = get_nodes(self.pos, msg, self.map.data) 
        nodes = all_nodes[2]
        start_node = all_nodes[0]
        end_node = all_nodes[1]

        width = self.map.width
        height = self.map.height


        if start_node is None or end_node is None:
            return
        
        positions = astar(nodes, start_node, end_node, (width, height))

        if positions is None:
            # there is no path from the start node to end node
            # wait for global costmap to update and try again
            self.no_path_found = True
            self.get_logger().info('No path found, will retry when global costmap updates')
            return
        
        path = create_path(positions)

        self.no_path_found = False

        # change to where it publishes the second point in path
        # for PID implementation
        self.path_pub.publish(path)

    
def main(args=None):
    
    rclpy.init(args=args)

    node = Astar()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()