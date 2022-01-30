# helpful resource: http://docs.ros.org/en/indigo/api/costmap_2d/html/costmap__2d_8h_source.html#l00171
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from virtuoso_msgs.msg import CostmapPoints, CostmapPointInfo
import math

class GetPoints(Node):

    def __init__(self):
        super().__init__('get_points')
        self.points_pub = self.create_publisher(CostmapPoints, '/path_finding/points', 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.get_points, 10)

    def get_points(self, msg:OccupancyGrid):

        points = CostmapPoints() 

        width = msg.info.width

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        resolution = msg.info.resolution

        for i, cost in enumerate(msg.data):

            point = CostmapPointInfo()
            
            p_x = i - (math.trunc((i / width)) * width)
            p_y = math.trunc(i / width)

            map_x = origin_x + (p_x * resolution)
            map_y = origin_y + (p_y * resolution)
            

            point.x = map_x
            point.y = map_y
            point.cost = cost

            points.data.append(point)
        
        points.width = width
        points.height = msg.info.height
        
        self.points_pub.publish(points)


def main(args=None):
    
    rclpy.init(args=args)

    node = GetPoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        