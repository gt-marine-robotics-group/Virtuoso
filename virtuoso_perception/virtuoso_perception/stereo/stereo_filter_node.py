import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from virtuoso_processing.utils.pointcloud import read_points, create_cloud_xyz32

class StereoFilterNode(Node):

    def __init__(self):
        super().__init__('perception_stereo_filter')

        self.pcd_sub = self.create_subscription(PointCloud2, '/perception/stereo/points',
            self.pcd_callback, 10)

        self.filtered_pub = self.create_publisher(PointCloud2, 
            '/perception/stereo/points_filtered', 10)
        
    def pcd_callback(self, msg:PointCloud2):

        points = list()

        for i, point in enumerate(read_points(msg)):
            if point[2] > -0.5:
                points.append(list(float('NaN') for _ in range(3)))
            else:
                points.append(list(point[i] for i in range(3)))
        
        filtered_cloud = create_cloud_xyz32(msg.header, points)

        self.filtered_pub.publish(filtered_cloud)


def main(args=None):

    rclpy.init(args=args)

    sub = StereoFilterNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()