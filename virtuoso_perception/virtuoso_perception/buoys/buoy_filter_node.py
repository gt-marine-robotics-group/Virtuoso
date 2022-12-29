import rclpy
from rclpy.node import Node
from virtuoso_perception.utils.ColorFilter import ColorFilter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ..utils.code_identification import find_contours
from ..utils.color_range import ColorRange
import numpy as np
from scipy import stats
import random
from virtuoso_msgs.msg import Contours
from .buoy_filter import BuoyFilter

class BuoyColorFilterNode(Node):

    def __init__(self):
        super().__init__('perception_buoy_filter')

        self.declare_parameters(namespace='', parameters=[
            ('debug', False),
            ('base_topic', ''), # comes from camera_config.yaml

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
            ('buoy_px_color_sample_size', 0)
        ])

        self.buoy_filter = BuoyFilter(color_filter_bounds=ColorRange(self, 
            ['red', 'green', 'black', 'yellow'], prefix='filter_bounds.'), 
            color_label_bounds=ColorRange(self, 
                ['red', 'green', 'black', 'yellow'], prefix='label_bounds.'),
            buoy_border_px=self.get_parameter('buoy_border_px').value,
            buoy_px_color_sample_size=self.get_parameter('buoy_px_color_sample_size').value
        )
        if self.get_parameter('debug').value:
            self.buoy_filter.node = self

        base_topic = self.get_parameter('base_topic').value

        self.image_sub = self.create_subscription(Image, 
            f'{base_topic}/resized', self.image_callback, 10)
        
        self.filter_pub = self.create_publisher(Contours,
            f'{base_topic}/buoy_filter', 10)
        
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

        return self.buoy_filter.contours
    
    def image_callback(self, msg:Image):
        self.filter_pub.publish(self.apply_filter(msg))


def main(args=None):
    
    rclpy.init(args=args)

    sub = BuoyColorFilterNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()