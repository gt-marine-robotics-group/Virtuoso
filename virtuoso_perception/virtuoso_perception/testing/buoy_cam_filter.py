import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from virtuoso_msgs.srv import ImageBuoyFilter

class TestBuoyCamFilterNode(Node):

    def __init__(self):
        super().__init__('test_buoy_cam_filter')

        self.declare_parameters(namespace='', parameters=[
            ('base_topic', '')
        ])

        base_topic = self.get_parameter('base_topic').value
        cam = base_topic[base_topic.rfind('/') + 1:]

        self.image_sub = self.create_subscription(Image,
            f'{base_topic}/image_raw', self.image_callback, 10)

        self.cam_info_sub = self.create_subscription(CameraInfo,
            f'{base_topic}/camera_info', self.cam_info_callback, 10)

        self.image = None
        self.cam_info = None

        self.client = self.create_client(ImageBuoyFilter, f'{cam}/buoy_filter')
        self.req = None

        self.create_timer(1.0, self.send_request)

    def send_request(self):
        if self.req is not None or self.image is None or self.cam_info is None:
            self.get_logger().info('something not none')
            return
        
        self.client.wait_for_service(2.0)

        msg = ImageBuoyFilter.Request()
        msg.image = self.image
        msg.camera_info = self.cam_info

        self.req = self.client.call_async(msg)
        self.req.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        self.req = None 
        self.get_logger().info('received response')
    
    def image_callback(self, msg:Image):
        self.image = msg
    
    def cam_info_callback(self, msg:CameraInfo):
        self.cam_info = msg


def main(args=None):
    rclpy.init(args=args)

    node = TestBuoyCamFilterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()