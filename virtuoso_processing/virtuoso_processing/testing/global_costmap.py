import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8MultiArray 

# Node that subscribes to global costmap and returns array of unique data values (ints)
# in costmap. 
# Used simply to get a better idea of how to implement navigation later. Not called in any launch file.

# Values found when tested were [-1, 0, 100]

class ViewCostmapPointTypes(Node):

    def __init__(self):
        super().__init__('global_costmap_points')
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.callback, 10)
        self.publisher = self.create_publisher(Int8MultiArray, '/global_costmap_points', 10)
        self.unique_vals = Int8MultiArray() 

    def callback(self, msg:OccupancyGrid):

        for val in msg.data:
            if val not in self.unique_vals.data:
                self.unique_vals.data.append(val) 
    
        self.publisher.publish(self.unique_vals)


def main(args=None):
    
    rclpy.init(args=args)

    node = ViewCostmapPointTypes()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

