import rclpy
from rclpy.node import Node
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

class ChannelNode(Node):

    def __init__(self):
        super().__init__('perception_channel')

        self.channel_srv = self.create_service(Channel, 'channel', 
            self.channel_callback)
        
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
    
    def activate_all(self, action=True):
        self.active = action
        self.cam_active_pub.publish(Bool(data=self.active))
        self.lidar_active_pub.publish(Bool(data=self.active))

    def find_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', "wamv/front_left_camera_link", Time()
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


def main(args=None):
    rclpy.init(args=args)

    node = ChannelNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()