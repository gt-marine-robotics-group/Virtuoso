import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8MultiArray

class GetPoints(Node):

    def __init__(self):
        super().__init__('get_points')
        self.points_pub = self.create_publisher(Int8MultiArray, '/path_finding/points', 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.get_points, 10)

    def get_points(self, msg:OccupancyGrid):

        points = []

        width = msg.info.width
        height = msg.info.height

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        resolution = msg.info.resolution

        for i, cost in enumerate(msg.data):
            
            p_x = i - ((i / width) * width)
            p_y = i / width

            map_x = origin_x + (p_x * resolution)
            map_y = origin_y + (p_y * resolution)
            
            points.append({'x': map_x, 'y': map_y, 'cost': cost})
        
        self.points_pub.publish(points)


def main(args=None):
    
    rclpy.init(args=args)

    node = GetPoints()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        