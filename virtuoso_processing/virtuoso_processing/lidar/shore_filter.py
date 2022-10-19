import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import math
from ..utils.pointcloud import read_points, create_cloud_xyz32
from robot_localization.srv import FromLL
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.duration import Duration
from rclpy.time import Time

class ShoreFilterer(Node):

    def __init__(self):
        super().__init__('shore_filter')

        self.lidar_sub = self.create_subscription(PointCloud2, '/processing/points_self_filtered', self.callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/processing/points_shore_filtered', 10)

    def callback(self, msg:PointCloud2):

        points = [] 

        for i, point in enumerate(read_points(msg)):
            points.append([0, 0, 0])
            if point[0] <= 0 or point[1] > 15 or point[1] < -15:
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

    node = ShoreFilterer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()