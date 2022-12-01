import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class StereoNode(Node):

    def __init__(self):
        super().__init__('processing_stereo')

        self.image_sub = self.create_subscription(Image, '/processing/image_downscaled', 
            self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, 
            '/wamv/sensors/cameras/front_left_camera/camera_info', self.cam_info_callback, 10)

        self.image = None 
        self.cam_info = None

        self.create_timer(0.1, self.execute)
    
    def image_callback(self, msg:Image):
        self.image = msg
    
    def cam_info_callback(self, msg:CameraInfo):
        self.cam_info = msg
    
    def execute(self):
        self.get_logger().info(str(self.cam_info))

def main(args=None):
    
    rclpy.init(args=args)

    sub = StereoNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()