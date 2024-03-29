#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from virtuoso_msgs.srv import DockCodesCameraPos, CountDockCodes, ImageBuoyFilter
from virtuoso_perception.camera_processing.resize import Resize
from virtuoso_perception.camera_processing.noise_filter import NoiseFilter
from virtuoso_perception.dock.find_dock_codes import FindDockCodes
from virtuoso_perception.utils.color_range import ColorRange
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class FindDockCodesNode(Node):

    def __init__(self):
        super().__init__('perception_find_dock_codes')

        self.cb_group1 = MutuallyExclusiveCallbackGroup()
        self.cb_group2 = MutuallyExclusiveCallbackGroup()

        self.declare_parameters(namespace='', parameters=[
            ('debug', False),
            ('camera_base_topic', ''),

            ('max_cluster_height', 0.0),
            ('min_cluster_height', 0.0),
            ('max_cluster_width', 0.0),
            ('min_cluster_width', 0.0),
            ('epsilon', 0),
            ('min_pts', 0),

            ('code_bounds.red.lower1', [0,0,0]),
            ('code_bounds.red.upper1', [0,0,0]),
            ('code_bounds.red.lower2', [0,0,0]),
            ('code_bounds.red.upper2', [0,0,0]),
            ('code_bounds.green.lower', [0,0,0]),
            ('code_bounds.green.upper', [0,0,0]),
            ('code_bounds.blue.lower', [0,0,0]),
            ('code_bounds.blue.upper', [0,0,0]),

            ('code_px_color_sample_size', 0.1),

            ('placard_bounds.lower', [0,0,0]),
            ('placard_bounds.upper', [0,0,0]),

            ('placard_color_search_range', 0),
            ('min_placard_color_prop_between_codes', 0.0),

            ('denoising_params', []),
            ('resize_factor', 1),

            ('use_resize', False),
            ('use_noise_filter', False)
        ])

        base_topic = self.get_parameter('camera_base_topic').value
        cam = base_topic[base_topic.rfind('/') + 1:]

        self.srv = self.create_service(DockCodesCameraPos, f'{cam}/find_dock_placard_offsets',   
            self.srv_callback, callback_group=self.cb_group1)
        
        self.count_srv = self.create_service(CountDockCodes, f'{cam}/count_dock_codes',
            self.count_srv_callback, callback_group=self.cb_group1)
        
        self.contour_srv = self.create_service(ImageBuoyFilter, f'{cam}/dock_code_contours',
            self.contour_callback, callback_group=self.cb_group1)
        
        self.image_sub = self.create_subscription(Image, f'{base_topic}/image_raw',
            self.image_callback, 10, callback_group=self.cb_group2)
        self.image:Image = None
        self.old_image = True

        self.camera_info_sub = self.create_subscription(CameraInfo, f'{base_topic}/camera_info',
            self.camera_info_callback, 10, callback_group=self.cb_group2)
        self.camera_info:CameraInfo = None
        self.cam_width = 0
        
        self.resize = Resize(resize_factor=self.get_parameter('resize_factor').value)
        self.noise_filter = NoiseFilter(denoising_params=self.get_parameter('denoising_params').value)

        if self.get_parameter('debug').value:
            node = self
        else: node = None
        self.find_dock_codes = FindDockCodes(max_cluster_height=self.get_parameter('max_cluster_height').value,
            min_cluster_height=self.get_parameter('min_cluster_height').value,
            max_cluster_width=self.get_parameter('max_cluster_width').value,
            min_cluster_width=self.get_parameter('min_cluster_width').value,
            epsilon=self.get_parameter('epsilon').value, min_pts=self.get_parameter('min_pts').value,
            code_px_color_sample_size=self.get_parameter('code_px_color_sample_size').value,
            code_color_bounds=ColorRange(self, ['red', 'green', 'blue'], prefix='code_bounds.'),
            placard_color_bounds={'lower': self.get_parameter('placard_bounds.lower').value,
                'upper': self.get_parameter('placard_bounds.upper').value}, 
            placard_prop=self.get_parameter('min_placard_color_prop_between_codes').value,
            placard_search_range=self.get_parameter('placard_color_search_range').value,
            node=node)

        self.debug_pubs = {
            'black_white': self.create_publisher(Image,
                '/find_dock_codes/debug/black_white', 10),
            'full_contours': self.create_publisher(Image,
                '/find_dock_codes/debug/full_contours', 10),
            'placard_bg_filter': self.create_publisher(Image,
                '/find_dock_codes/debug/placard_bg_filter', 10),
            'codes': self.create_publisher(Image, 
                '/find_dock_codes/debug/codes', 10)
        }

    def debug_pub(self, name:str, msg):
        self.debug_pubs[name].publish(msg)
    
    def image_callback(self, msg:Image):
        self.image = msg
        self.old_image = False
    
    def camera_info_callback(self, msg:CameraInfo):
        self.camera_info = msg
        self.cam_width = msg.width // self.get_parameter('resize_factor').value
    
    def process_image(self):
        self.resize.image = self.image
        self.resize.camera_info = self.camera_info

        if self.get_parameter('use_resize').value:
            self.get_logger().info('resizing')
            image, camera_info = self.resize.resize()
        else:
            image = self.image
            camera_info = self.camera_info

        self.noise_filter.image = image
        
        if self.get_parameter('use_noise_filter').value:
            self.get_logger().info('noise filtering')
            image = self.noise_filter.filter()

        return image, camera_info
    
    def count_srv_callback(self, req:CountDockCodes.Request, res:CountDockCodes.Response):
        self.get_logger().info('service called')

        if self.image is None:
            self.get_logger().info('No image')
            return res
        
        if self.camera_info is None:
            self.get_logger().info('No camera info')
            return res

        while self.old_image:
            self.get_logger().info('Waiting for new image')
            time.sleep(0.5)
        
        self.old_image = True

        image, camera_info = self.process_image()

        self.find_dock_codes.image = image
        count = self.find_dock_codes.run(search='COUNT', search_color=req.color)

        res.count = count

        return res

    def contour_callback(self, req:ImageBuoyFilter.Request, res:ImageBuoyFilter.Response):
        self.get_logger().info('service called')

        if self.image is None:
            self.get_logger().info('No image')
            return res
        
        if self.camera_info is None:
            self.get_logger().info('No camera info')
            return res

        while self.old_image:
            self.get_logger().info('Waiting for new image')
            time.sleep(0.5)
        
        self.old_image = True

        image, camera_info = self.process_image()

        self.find_dock_codes.image = image
        contours, offsets, colors = self.find_dock_codes.run(search='CODE_CONTOURS')


        res.contours.contour_points = contours
        res.contours.contour_offsets = offsets
        res.contours.contour_colors = colors
        res.camera_info = camera_info

        return res
    
    def srv_callback(self, req:DockCodesCameraPos.Request, res:DockCodesCameraPos.Response):
        self.get_logger().info('service called')

        if self.image is None:
            self.get_logger().info('No image')
            return res
        
        if self.camera_info is None:
            self.get_logger().info('No camera info')
            return res

        while self.old_image:
            self.get_logger().info('Waiting for new image')
            time.sleep(0.5)
        
        self.old_image = True

        image, camera_info = self.process_image()

        self.find_dock_codes.image = image
        bounds = self.find_dock_codes.run()

        res.image_width = self.cam_width
        res.red = bounds['red']
        res.blue = bounds['blue']
        res.green = bounds['green']

        return res

def main(args=None):
    rclpy.init(args=args)

    node = FindDockCodesNode()

    executor = MultiThreadedExecutor(num_threads=2)

    executor.add_node(node)

    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()