import rclpy
from rclpy.node import Node

class CameraInfoNode(Node):

    def __init__(self):
        super().__init__('sensors_camera_info')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', ''),
            ('matrix', [])
        ])


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