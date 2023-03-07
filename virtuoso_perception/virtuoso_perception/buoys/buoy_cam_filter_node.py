import rclpy
from rclpy.client import Client
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ..utils.code_identification import find_contours
from ..utils.color_range import ColorRange
from ..utils.image_srv_chain import ImageSrvChain
from .buoy_cam_filter import BuoyFilter
from virtuoso_msgs.srv import ImageNoiseFilter, ImageResize, ImageBuoyFilter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from ..camera_processing.noise_filter import NoiseFilter
from ..camera_processing.resize import Resize

class BuoyFilterNode(Node):

    def __init__(self):
        super().__init__('perception_buoy_filter')

        self.declare_parameters(namespace='', parameters=[
            ('debug', False),
            ('base_topic', ''),

            ('filter_bounds.red.lower1', [0,0,0]),
            ('filter_bounds.red.upper1', [0,0,0]),
            ('filter_bounds.red.lower2', [0,0,0]),
            ('filter_bounds.red.upper2', [0,0,0]),
            ('filter_bounds.green.lower', [0,0,0]),
            ('filter_bounds.green.upper', [0,0,0]),
            ('filter_bounds.yellow.lower', [0,0,0]),
            ('filter_bounds.yellow.upper', [0,0,0]),
            ('filter_bounds.black.lower', [0,0,0]),
            ('filter_bounds.black.upper', [0,0,0]),

            ('label_bounds.red.lower1', [0,0,0]),
            ('label_bounds.red.upper1', [0,0,0]),
            ('label_bounds.red.lower2', [0,0,0]),
            ('label_bounds.red.upper2', [0,0,0]),
            ('label_bounds.green.lower', [0,0,0]),
            ('label_bounds.green.upper', [0,0,0]),
            ('label_bounds.yellow.lower', [0,0,0]),
            ('label_bounds.yellow.upper', [0,0,0]),
            ('label_bounds.black.lower', [0,0,0]),
            ('label_bounds.black.upper', [0,0,0]),

            ('buoy_border_px', 0),
            ('buoy_px_color_sample_size', 0),

            ('denoising_params', []),
            ('resize_factor', 1)
        ])

        base_topic = self.get_parameter('base_topic').value
        cam = base_topic[base_topic.rfind('/') + 1:]
        
        self.srv = self.create_service(ImageBuoyFilter, 
            f'{cam}/buoy_filter', self.srv_callback)

        self.resize = Resize(resize_factor=self.get_parameter('resize_factor').value)
        self.noise_filter = NoiseFilter(denoising_params=self.get_parameter('denoising_params').value)

        self.buoy_filter = BuoyFilter(color_filter_bounds=ColorRange(self, 
            ['red', 'green', 'black', 'yellow'], prefix='filter_bounds.'), 
            color_label_bounds=ColorRange(self, 
                ['red', 'green', 'black', 'yellow'], prefix='label_bounds.'),
            buoy_border_px=self.get_parameter('buoy_border_px').value,
            buoy_px_color_sample_size=self.get_parameter('buoy_px_color_sample_size').value
        )
        if self.get_parameter('debug').value:
            self.buoy_filter.node = self

        self.debug_pubs = {
            'black_white': self.create_publisher(Image,
            f'{base_topic}/buoy_filter/debug/black_white', 10),
            'full_contours': self.create_publisher(Image,
            f'{base_topic}/buoy_filter/debug/full_contours', 10),
            'filtered_contours': self.create_publisher(Image,
            f'{base_topic}/buoy_filter/debug/filtered_contours', 10)
        }

    def apply_filter(self, img:Image):
        bgr_image = CvBridge().imgmsg_to_cv2(img, desired_encoding='bgr8')

        self.buoy_filter.image = bgr_image
        self.buoy_filter.run()

        self.buoy_filter.contours.header.stamp = self.get_clock().now().to_msg()

        return self.buoy_filter.contours
    
    def srv_callback(self, req:ImageBuoyFilter.Request, res:ImageBuoyFilter.Response):

        if req.image is None or req.camera_info is None:
            return res
        
        self.resize.image = req.image
        self.resize.camera_info = req.camera_info

        image, camera_info = self.resize.resize()

        self.noise_filter.image = image
        
        image = self.noise_filter.filter()
        
        res.contours = self.apply_filter(image)
        res.camera_info = camera_info

        return res


def main(args=None):
    
    rclpy.init(args=args)

    node = BuoyFilterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()