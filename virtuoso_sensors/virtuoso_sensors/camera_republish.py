import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CameraRepublish(Node):
    
    def __init__(self):
        super().__init__('camera_republish')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(Image, 'output', qos_profile=qos_profile)

        self.sub = self.create_subscription(Image, 'input', self.callback, 10)

    def callback(self, msg: Image):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    camera_republish = CameraRepublish()

    rclpy.spin(camera_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically)
    # when the garbage collector destroys the node object
    camera_republish.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
