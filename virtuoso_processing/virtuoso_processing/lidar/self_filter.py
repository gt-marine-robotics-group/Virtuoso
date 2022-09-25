import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import math
from ..utils.pointcloud import read_points, create_cloud_xyz32

class SelfFilter(Node):

    def __init__(self):
        super().__init__('self_filter')
        # self.lidar_sub = self.create_subscription(PointCloud2, '/points_shore_filtered', self.callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/points_nonground', self.callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/points_self_filtered', 10)

    def callback(self, msg:PointCloud2):

        points = [] 

        for i, point in enumerate(read_points(msg)):
            points.append([0, 0, 0])
            if(math.sqrt((point[0]**2) + (point[1]**2)) < .9):
                points[i][0] = float("NaN")
                points[i][1] = float("NaN")
                points[i][2] = float("NaN")
            else:
                points[i][0] = point[0] 
                points[i][1] = point[1]
                points[i][2] = point[2] 
        
        
        filtered_cloud = create_cloud_xyz32(msg.header, points)

        self.publisher.publish(filtered_cloud)



def main(args=None):
    
    rclpy.init(args=args)

    node = SelfFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
