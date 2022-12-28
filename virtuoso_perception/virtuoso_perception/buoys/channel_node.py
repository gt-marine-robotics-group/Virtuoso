import rclpy
from rclpy.node import Node
from virtuoso_msgs.srv import Channel
from virtuoso_msgs.msg import BuoyArray, Buoy
from geometry_msgs.msg import Point, TransformStamped, PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from typing import List
import math
from ..utils.geometry_msgs import do_transform_point

class ChannelNode(Node):

    def __init__(self):
        super().__init__('perception_channel')

        self.channel_srv = self.create_service(Channel, 'channel', 
            self.channel_callback)
        
        self.buoys_sub = self.create_subscription(BuoyArray, '/perception/stereo/buoys',
            self.buoys_callback, 10)
        
        self.buoys:BuoyArray = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def find_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', "wamv/front_left_camera_link", Time()
            )
            return trans
        except Exception:
            return None

    def buoys_callback(self, msg:BuoyArray):
        self.buoys = msg
    
    def find_closest_buoy(buoys:List[Buoy]):
        return min(buoys, key=lambda b: math.sqrt(b.location.x**2 + b.location.y**2))
    
    def transform_point(trans:TransformStamped, point:Point):
        ps = PointStamped(point=point)

        trans_ps = do_transform_point(ps, trans)

        return trans_ps.point
    
    def channel_callback(self, req:Channel.Request, res:Channel.Response):
        res.header.frame_id = 'map'
        res.left = Point(x=0.0,y=0.0,z=0.0)
        res.right = Point(x=0.0,y=0.0,z=0.0)

        if self.buoys is None:
            return res
        
        trans = self.find_transform()
        if trans is None:
            return res
        
        left_buoys:List[Buoy] = list()
        right_buoys:List[Buoy] = list()
        for buoy in self.buoys.buoys:
            if math.sqrt(buoy.location.x**2 + buoy.location.y**2) > req.max_dist_from_usv:
                continue
            if buoy.color == req.left_color:
                left_buoys.append(buoy)
            elif buoy.color == req.right_color:
                right_buoys.append(buoy)
        
        if len(left_buoys) > 0:
            closest = ChannelNode.find_closest_buoy(left_buoys)
            res.left.x = closest.location.x
            res.left.y = closest.location.y
            res.left = ChannelNode.transform_point(trans, res.left)
        
        if len(right_buoys) > 0:
            closest = ChannelNode.find_closest_buoy(right_buoys)
            res.right.x = closest.location.x
            res.right.y = closest.location.y
            res.right = ChannelNode.transform_point(trans, res.right)
        
        return res


def main(args=None):
    rclpy.init(args=args)

    node = ChannelNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()