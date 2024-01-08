import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class cameraRepublish(Node):
    
    def __init__(self):
        super().__init__('camera_republish')

        self.front_left_image = Image() # Image could be an incorrect data type
        self.front_right_image = Image()

        self.front_left_camera_ready = False
        self.front_right_camera_ready = False

        self.frontLeftCameraPublisher = self.create_publisher(Image, '/cameras/front_left_camera', 10)
        self.frontRightCameraPublisher = self.create_publisher(Image, '/cameras/front_right_camera', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliability.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.front_left_camera_subscriber = self.create_subscription(
            Image,
            'cameras/front_left_camera/image_raw',
            self.front_left_camera_callback,
            qos_profile=qos_profile
        )
        self.front_right_camera_subscriber = self.create_subscription(
            Image,
            'cameras/front_right_camera/image_raw',
            self.front_right_camera_callback,
            qos_profile=qos_profile
        )

    def front_left_camera_callback(self, msg):
        self.front_left_image = msg
        self.front_left_camera_ready = True
        self.publish_camera()

    def front_right_camera_callback(self, msg):
        self.front_right_image = msg
        self.front_right_camera_ready = True
        self.publish_camera()

    def publish_camera(self):


        if (self.front_left_camera_ready):
            self.frontLeftCameraPublisher.publish(front_left_image)
            self.front_left_camera_ready = False

        if (self.front_right_camera_ready):
            self.frontRightCameraPublisher.publish(front_right_image)
            self.front_right_camera_ready = False

def main(args=None):
    rclpy.init(args=args)

    camera_republish = cameraRepublish()

    rclpy.spin(camera_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically)
    # when the garbage collector destroys the node object
    camera_republish.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
