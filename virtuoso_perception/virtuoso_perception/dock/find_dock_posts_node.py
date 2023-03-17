import rclpy
from rclpy.node import Node
from ..camera_processing.resize import Resize
from ..camera_processing.noise_filter import NoiseFilter
from .find_dock_posts import FindDockPosts
from ..utils.color_range import ColorRange
from virtuoso_msgs.srv import ImageBuoyFilter
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class FindDockPostsNode(Node):

    def __init__(self):
        super().__init__('perception_find_dock_posts')

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

            ('post_bounds.red.lower1', [0,0,0]),
            ('post_bounds.red.upper1', [0,0,0]),
            ('post_bounds.red.lower2', [0,0,0]),
            ('post_bounds.red.upper2', [0,0,0]),
            ('post_bounds.green.lower', [0,0,0]),
            ('post_bounds.green.upper', [0,0,0]),
            ('post_bounds.white.lower', [0,0,0]),
            ('post_bounds.white.upper', [0,0,0]),

            ('post_px_color_sample_size', 0.1),
            ('post_min_px_density', 0.0),

            ('placard_bounds.lower', [0,0,0]),
            ('placard_bounds.upper', [0,0,0]),

            ('placard_color_search_range', 0),
            ('max_placard_prop', 0.0),

            ('denoising_params', []),
            ('resize_factor', 1),

            ('use_resize', False),
            ('use_noise_filter', False)
        ])

        base_topic = self.get_parameter('camera_base_topic').value
        cam = base_topic[base_topic.rfind('/') + 1:]

        self.srv = self.create_service(ImageBuoyFilter, f'{cam}/find_dock_posts', 
            self.srv_callback, callback_group=self.cb_group1)
        
        self.image_sub = self.create_subscription(Image, f'{base_topic}/image_raw',
            self.image_callback, 10, callback_group=self.cb_group2)
        self.image:Image = None
        self.old_image = True

        self.camera_info_sub = self.create_subscription(CameraInfo, f'{base_topic}/camera_info',
            self.camera_info_callback, 10, callback_group=self.cb_group2)
        self.camera_info:CameraInfo = None
        self.old_cam_info = True
        
        self.resize = Resize(resize_factor=self.get_parameter('resize_factor').value)
        self.noise_filter = NoiseFilter(denoising_params=self.get_parameter('denoising_params').value)

        if self.get_parameter('debug').value:
            node = self
        else: node = None
        self.find_dock_posts = FindDockPosts(max_cluster_height=self.get_parameter('max_cluster_height').value,
            min_cluster_height=self.get_parameter('min_cluster_height').value,
            max_cluster_width=self.get_parameter('max_cluster_width').value,
            min_cluster_width=self.get_parameter('min_cluster_width').value,
            epsilon=self.get_parameter('epsilon').value, min_pts=self.get_parameter('min_pts').value,
            post_px_color_sample_size=self.get_parameter('post_px_color_sample_size').value,
            post_px_density=self.get_parameter('post_min_px_density').value,
            post_color_bounds=ColorRange(self, ['red', 'green', 'white'], prefix='post_bounds.'),
            placard_color_bounds={'lower': self.get_parameter('placard_bounds.lower').value,
                'upper': self.get_parameter('placard_bounds.upper').value},
            placard_prop=self.get_parameter('max_placard_prop').value,
            placard_search_range=self.get_parameter('placard_color_search_range').value,
            node=node)

        self.debug_pubs = {
            'black_white': self.create_publisher(Image,
                '/find_dock_posts/debug/black_white', 10),
            'full_contours': self.create_publisher(Image,
                '/find_dock_posts/debug/full_contours', 10),
            'placard_bg_filter': self.create_publisher(Image,
                '/find_dock_posts/debug/placard_bg_filter', 10),
            'posts': self.create_publisher(Image, 
                '/find_dock_posts/debug/posts', 10)
        }

    def debug_pub(self, name:str, msg):
        self.debug_pubs[name].publish(msg)
    
    def image_callback(self, msg:Image):
        self.image = msg
        self.old_image = False
    
    def camera_info_callback(self, msg:CameraInfo):
        self.camera_info = msg
        self.old_cam_info = False
    
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
    
    def srv_callback(self, req:ImageBuoyFilter.Request, res:ImageBuoyFilter.Response):
        self.get_logger().info('service called')

        if self.image is None:
            self.get_logger().info('No image')
            return res
        
        if self.camera_info is None:
            self.get_logger().info('No camera info')
            return res

        while self.old_image or self.old_cam_info:
            self.get_logger().info('Waiting for new image or waiting for cam info')
            time.sleep(0.5)
        
        self.old_image = True
        self.old_cam_info = True

        image, camera_info = self.process_image()

        self.find_dock_posts.image = image
        contours, offsets, colors = self.find_dock_posts.run()

        res.camera_info = camera_info
        res.contours.contour_points = contours
        res.contours.contour_offsets = offsets
        res.contours.contour_colors = colors

        return res

def main(args=None):
    rclpy.init(args=args)

    node = FindDockPostsNode()

    executor = MultiThreadedExecutor(num_threads=2)

    executor.add_node(node)

    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()