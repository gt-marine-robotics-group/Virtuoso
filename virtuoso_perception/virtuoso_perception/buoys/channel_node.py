import rclpy
from rclpy.node import Node
import time
from virtuoso_msgs.srv import Channel
from virtuoso_msgs.msg import BuoyArray
from geometry_msgs.msg import Point
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from std_msgs.msg import Bool
from .channel import FindChannel
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from virtuoso_msgs.srv import ImageBuoyFilter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ChannelNode(Node):

    def __init__(self):
        super().__init__('perception_channel')

        self.cb_group_1 = MutuallyExclusiveCallbackGroup()
        self.cb_group_2 = MutuallyExclusiveCallbackGroup()
        self.cb_group_3 = MutuallyExclusiveCallbackGroup()

        self.test = None
        self.camera_info = None
        self.test_img1 = None
        self.test_img2 = None

        self.declare_parameters(namespace='', parameters=[
            ('camera_frame', '')
        ])

        self.test_sub = self.create_subscription(Image, 
            '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.test_callback, 10)
        self.camera_info = self.create_subscription(CameraInfo, 
            '/wamv/sensors/cameras/front_left_camera/camera_info', 
            self.camera_info_callback, 10)

        self.channel_srv = self.create_service(Channel, 'channel', 
            self.channel_callback, callback_group=self.cb_group_1)

        # self.test_client = ActionClient(self, ImageNoiseFilter, 
        #     'perception/image_noise_filter', callback_group=self.cb_group_2)
        # self.test_client2 = ActionClient(self, ImageNoiseFilter,
        #     'perception/image_noise_filter', callback_group=self.cb_group_3)
        self.test_client = self.create_client(ImageBuoyFilter, 
            'perception/image_buoy_filter', callback_group=self.cb_group_2)
        self.test_client2 = self.create_client(ImageBuoyFilter, 
            'perception/image_buoy_filter', callback_group=self.cb_group_3)
        
        self.cam_active_pub = self.create_publisher(Bool, 
            '/perception/camera/activate_processing', 10)
        self.lidar_active_pub = self.create_publisher(Bool,
            '/perception/buoys/buoy_lidar/activate', 10)
        
        self.camera_buoys_sub = self.create_subscription(BuoyArray, 
            '/perception/stereo/buoys', self.camera_buoys_callback, 10)
        self.lidar_buoys_sub = self.create_subscription(BoundingBoxArray,
            '/buoys/bounding_boxes', self.lidar_buoys_callback, 10)

        self.odom_sub = self.create_subscription(Odometry, 
            '/localization/odometry', self.odom_callback, 10)
        
        self.channel = FindChannel()
        self.channel.node = self
        
        self.active = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def test_callback(self, msg:Image):
        self.test = msg
    
    def camera_info_callback(self, msg:CameraInfo):
        self.camera_info = msg
    
    def activate_all(self, action=True):
        self.active = action
        self.cam_active_pub.publish(Bool(data=self.active))
        self.lidar_active_pub.publish(Bool(data=self.active))

    def find_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', self.get_parameter('camera_frame').value, Time()
            )
            return trans
        except Exception:
            return None

    def camera_buoys_callback(self, msg:BuoyArray):
        if not self.active:
            return
        self.channel.camera_buoys = msg
    
    def lidar_buoys_callback(self, msg:BoundingBoxArray):
        if not self.active:
            return
        self.channel.lidar_buoys = msg
    
    def odom_callback(self, msg:Odometry):
        self.channel.odom = msg
    
    def channel_callback(self, req:Channel.Request, res:Channel.Response):
        res.header.frame_id = 'map'
        res.left = Point(x=0.0,y=0.0,z=0.0)
        res.right = Point(x=0.0,y=0.0,z=0.0)

        msg1 = ImageBuoyFilter.Request()
        msg1.image = self.test
        msg1.camera_info = self.camera_info
        msg2 = ImageBuoyFilter.Request()
        msg2.image = self.test
        msg2.camera_info = self.camera_info
        self.get_logger().info('waiting for service')
        self.test_client.wait_for_service(timeout_sec=2.0)
        self.get_logger().info('sending async requests')
        gf1 = self.test_client.call_async(msg1)
        gf1.add_done_callback(self.gf1_cb2)
        gf2 = self.test_client2.call_async(msg2)
        gf2.add_done_callback(self.gf2_cb2)

        while self.test_img1 is None or self.test_img2 is None:
            self.get_logger().info('channel node waiting for images')
            time.sleep(0.5)
        
        self.get_logger().info('sending response')

        return res

        if not self.active:
            self.activate_all()
            return res
        
        trans = self.find_transform()
        if trans is None:
            return res
        self.channel.cam_to_map_trans = trans
        
        res = self.channel.execute(req, res)
        
        if res.left == res.right:
            return res
        
        self.channel.reset()
        self.active = False
        self.activate_all(action=False)
        
        return res

    def gf1_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info('goal not accepted')
            return
        self.get_logger().info('goal accepted')

        self.f1 = handle.get_result_async()
        self.f1.add_done_callback(self.gf1_cb2)
    
    def gf1_cb2(self, future):
        self.get_logger().info('gf1 done')
        self.test_img1 = future.result().contours

    def gf2_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info('goal not accepted')
            return
        self.get_logger().info('goal accepted')

        self.f1 = handle.get_result_async()
        self.f1.add_done_callback(self.gf2_cb2)
    
    def gf2_cb2(self, future):
        self.get_logger().info('gf2 done')
        # self.get_logger().info(f'message: {future.result()}')
        self.test_img2 = future.result().contours


def main(args=None):
    rclpy.init(args=args)

    node = ChannelNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()