import rclpy
from rclpy.node import Node
from virtuoso_msgs.srv import DockPlacardCameraPos
from ..camera_processing.resize import Resize
from ..camera_processing.noise_filter import NoiseFilter
from .find_dock_codes import FindDockCodes
from ..utils.color_range import ColorRange
from sensor_msgs.msg import Image, CameraInfo

class FindDockCodesNode(Node):

    def __init__(self):
        super().__init__('perception_find_dock_codes')

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

            ('placard_bounds.lower', [0,0,0]),
            ('placard_bounds.upper', [0,0,0]),

            ('min_placard_color_prop_between_codes', 0.0),

            ('denoising_params', []),
            ('resize_factor', 1),

            ('use_resize', False),
            ('use_noise_filter', False)
        ])

        base_topic = self.get_parameter('camera_base_topic').value

        self.srv = self.create_service(DockPlacardCameraPos, 'find_dock_placard_offsets',   
            self.srv_callback)
        
        self.get_logger().info(f'{base_topic}/image_raw')
        self.image_sub = self.create_subscription(Image, f'{base_topic}/image_raw',
            self.image_callback, 10)
        self.image:Image = None
        
        self.resize = Resize(resize_factor=self.get_parameter('resize_factor').value)
        self.noise_filter = NoiseFilter(denoising_params=self.get_parameter('denoising_params').value)

        self.find_dock_codes = FindDockCodes(max_cluster_height=self.get_parameter('max_cluster_height').value,
            min_cluster_height=self.get_parameter('min_cluster_height').value,
            max_cluster_width=self.get_parameter('max_cluster_width').value,
            min_cluster_width=self.get_parameter('min_cluster_width').value,
            epsilon=self.get_parameter('epsilon').value, min_pts=self.get_parameter('min_pts').value, 
            code_color_bounds=ColorRange(self, ['red', 'green', 'blue'], prefix='code_bounds.'),
            placard_color_bounds={'lower': self.get_parameter('placard_bounds.lower').value,
                'upper': self.get_parameter('placard_bounds.upper').value}, 
            placard_prop=self.get_parameter('min_placard_color_prop_between_codes').value)

        if self.get_parameter('debug').value:
            self.find_dock_codes.node = self
    
    def image_callback(self, msg:Image):
        self.get_logger().info('image callback')
        self.image = msg
    
    def srv_callback(self, req:DockPlacardCameraPos.Request, res:DockPlacardCameraPos.Response):
        self.get_logger().info('service called')

        return res

def main(args=None):
    rclpy.init(args=args)

    node = FindDockCodesNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()