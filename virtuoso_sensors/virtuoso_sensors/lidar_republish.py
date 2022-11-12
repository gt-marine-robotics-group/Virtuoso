import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from virtuoso_processing.utils.pointcloud import read_points, create_cloud_xyz32, create_cloud

class LidarRepublish(Node):

    def __init__(self):
        super().__init__('lidar_republish')

        self.sub = self.create_subscription(PointCloud2, '/velodyne_points', self.republish, 10)

        self.pub = self.create_publisher(PointCloud2, 'wamv/sensors/lidars/lidar_wamv/points', 10)
    
    def republish(self, msg:PointCloud2):

        # newMsg = msg
        # newMsg.header.frame_id = 'wamv/lidar_wamv_link'
        
        newMsg = msg
        points = list()

        for i, point in enumerate(read_points(msg)):
            points.append([0, 0, 0, 0])
            points[i][0] = point[0] 
            points[i][1] = point[1]
            points[i][2] = point[2] 
        
        # newMsg = msg
        newMsg.header.frame_id = 'wamv/lidar_wamv_link'
        # newMsg.fields = newMsg.fields[0:3]

        newMsg = create_cloud(newMsg.header, newMsg.fields[0:4], points)
            
        # newMsg = (newMsg.header, points)

        self.pub.publish(newMsg)

def main(args=None):
    rclpy.init(args=args)

    lidar_republish = LidarRepublish()

    rclpy.spin(lidar_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        