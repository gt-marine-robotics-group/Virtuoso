import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CameraRepublish(Node):

    def __init__(self):
        super().__init__('camera_republish')

        profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            depth=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
        )

        self.left_sub = self.create_subscription(Image, '/cameras/front_left_camera/image_raw', 
            self.left_republish, profile)
        
        self.right_sub = self.create_subscription(Image, '/cameras/front_right_camera/image_raw', 
            self.right_republish, profile)

        self.left_pub = self.create_publisher(Image, '/debug/cameras/front_left_camera/image_raw',
            profile)

        self.right_pub = self.create_publisher(Image, '/debug/cameras/front_right_camera/image_raw',
            profile)

        self.pub = self.create_publisher(Image, 'wamv/sensors/cameras/front_left_camera/image_raw', 10)
    
    def left_republish(self, msg:Image):
        self.left_pub.publish(msg)
    
    def right_republish(self, msg:Image):
        self.right_pub.publish(msg)


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
        