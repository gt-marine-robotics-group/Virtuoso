import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ScanCode(Node):

    def __init__(self):
        super().__init__('scan_code')

        self.camera_sub = self.create_subscription(Image, 'downscaled_image', self.image_callback, 10)

        # scan twice (or more) to verify code is correct before publishing
        self.codes = [] # [['red', 'green', 'blue], ['red', 'green', 'blue']]
        self.is_black = False # code resets when it goes black

    def image_callback(self, msg:Image):
        pass
        


def main(args=None):
    rclpy.init(args=args)
    node = ScanCode
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()