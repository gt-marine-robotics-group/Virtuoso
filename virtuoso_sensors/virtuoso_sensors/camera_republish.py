import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraRepublish(Node):

    def __init__(self):
        super().__init__('lidar_republish')

        self.sub = self.create_subscription(Image, '/image_raw', self.republish, 10)

        self.pub = self.create_publisher(Image, 'wamv/sensors/cameras/front_left_camera/image_raw', 10)
    
    def republish(self, msg:Image):

        newMsg = msg
        newMsg.header.frame_id = 'wamv/front_left_camera_link_optical'

        self.pub.publish(newMsg)

def main(args=None):
    rclpy.init(args=args)

    camera_republish = CameraRepublish()

    rclpy.spin(camera_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        