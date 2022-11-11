import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LidarRepublish(Node):

    def __init__(self):
        super().__init__('lidar_republish')

        self.sub = self.create_subscription(PointCloud2, '/velodyne_points', self.republish, 10)

        self.pub = self.create_publisher(PointCloud2, 'wamv/sensors/lidars/lidar_wamv/points', 10)
    
    def republish(self, msg:PointCloud2):

        newMsg = msg
        newMsg.header.frame_id = 'wamv/lidar_wamv_link'

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
        