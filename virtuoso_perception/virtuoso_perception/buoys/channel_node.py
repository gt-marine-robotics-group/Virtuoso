import rclpy
from rclpy.node import Node
import time
from virtuoso_msgs.srv import Channel
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from std_msgs.msg import Bool
from .channel import FindChannel
from virtuoso_msgs.srv import ImageBuoyStereo, LidarBuoy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ChannelNode(Node):

    def __init__(self):
        super().__init__('perception_channel')

        self.cb_group_1 = MutuallyExclusiveCallbackGroup()
        self.cb_group_2 = MutuallyExclusiveCallbackGroup()
        self.cb_group_3 = MutuallyExclusiveCallbackGroup()

        self.declare_parameters(namespace='', parameters=[
            ('camera_frame', '')
        ])

        self.channel_srv = self.create_service(Channel, 'channel', 
            self.channel_callback, callback_group=self.cb_group_1)

        self.stereo_client = self.create_client(ImageBuoyStereo, 
            'perception/image_buoy_stereo', callback_group=self.cb_group_2)
        self.lidar_client = self.create_client(LidarBuoy, 
            'perception/lidar_buoy', callback_group=self.cb_group_3)
        
        self.odom_sub = self.create_subscription(Odometry, 
            '/localization/odometry', self.odom_callback, 10)
        
        self.channel = FindChannel()
        self.channel.node = self
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def find_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', self.get_parameter('camera_frame').value, Time()
            )
            return trans
        except Exception:
            return None

    def odom_callback(self, msg:Odometry):
        self.channel.odom = msg
    
    def channel_callback(self, req:Channel.Request, res:Channel.Response):
        res.header.frame_id = 'map'
        res.left = Point(x=0.0,y=0.0,z=0.0)
        res.right = Point(x=0.0,y=0.0,z=0.0)

        self.channel.camera_buoys = None
        self.channel.lidar_buoys = None

        if req.use_camera:
            stereo_msg = ImageBuoyStereo.Request()
            stereo_call = self.stereo_client.call_async(stereo_msg)
            stereo_call.add_done_callback(self.stereo_callback)

        if req.use_lidar:
            lidar_msg = LidarBuoy.Request()
            lidar_call = self.lidar_client.call_async(lidar_msg)
            lidar_call.add_done_callback(self.lidar_callback)

        while ((req.use_camera and self.channel.camera_buoys is None) or 
            (req.use_lidar and self.channel.lidar_buoys is None)):
            time.sleep(0.5)
        
        trans = self.find_transform()
        if trans is None:
            return res
        self.channel.cam_to_map_trans = trans
        
        try:
            res = self.channel.execute(req, res)
        except Exception as e:
            self.get_logger().info(f'channel error: {e}')
            self.channel.reset()
            return res
        
        if res.left == res.right:
            return res
        
        self.channel.reset()
        
        return res

    def stereo_callback(self, future):
        result = future.result()
        self.channel.camera_buoys = result.buoys
    
    def lidar_callback(self, future):
        result = future.result()
        self.channel.lidar_buoys = result.buoys


def main(args=None):
    rclpy.init(args=args)

    node = ChannelNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()