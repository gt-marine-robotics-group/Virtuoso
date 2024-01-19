import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoNode(Node):

    def __init__(self):
        super().__init__('sensors_camera_info')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
            ('frame', ''),
            ('matrix', [0.0])
        ])

        self.empty_info_sub = self.create_subscription(CameraInfo, 
            f'{self.get_parameter("base_topic").value}/empty_camera_info',
            self.callback, 10)

        self.info_pub = self.create_publisher(CameraInfo,
            f'{self.get_parameter("base_topic").value}/camera_info', 10)
        
    def callback(self, msg:CameraInfo):
        msg.header.frame_id = self.get_parameter('frame').value
        msg.k = self.get_parameter('matrix').value

        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    camera_republish = CameraInfoNode()

    rclpy.spin(camera_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()