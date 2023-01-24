import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from .laser_geometry import LaserProjection

class LaserToPcdNode(Node):

    def __init__(self):
        super().__init__('laser_to_pcd')

        self.declare_parameter('frame', '')

        self.input_sub = self.create_subscription(LaserScan, '/scan', 
            self.input_callback, 10)
        
        self.output_pub = self.create_publisher(PointCloud2, '/output', 10)
    
    def input_callback(self, msg:LaserScan):

        pcd:PointCloud2 = LaserProjection().projectLaser(msg, 
            channel_options=LaserProjection.ChannelOption.INTENSITY)

        pcd.header.frame_id = self.get_parameter('frame').value

        self.output_pub.publish(pcd)


def main(args=None):
    rclpy.init(args=args)

    camera_republish = LaserToPcdNode()

    rclpy.spin(camera_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()