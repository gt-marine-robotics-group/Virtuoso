from geometry_msgs.msg import TransformStamped, Point, PointStamped
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from virtuoso_msgs.msg import BuoyArray, Buoy
from virtuoso_msgs.srv import Channel
from typing import List, Tuple
import math
from ..utils.geometry_msgs import do_transform_point

class FindChannel: 

    null_point = Point(x=0.0,y=0.0,z=0.0)
    
    def __init__(self):

        self.cam_to_map_trans:TransformStamped = None
        
        self.lidar_buoys:BoundingBoxArray = None
        self.camera_buoys:BuoyArray = None

        # When count becomes greater than some number, just fall back to 
        # using LIDAR only if that is an option
        self.iteration_count = 0

    def _find_closest_buoy(buoys:List[Buoy]):
        return min(buoys, key=lambda b: math.sqrt(b.location.x**2 + b.location.y**2))
    
    def _transform_point(trans:TransformStamped, point:Point):
        ps = PointStamped(point=point)

        trans_ps = do_transform_point(ps, trans)

        return trans_ps.point
    
    def reset(self):
        self.lidar_buoys = None
        self.camera_buoys = None
        self.iteration_count = 0
    
    def execute(self, req:Channel.Request, res:Channel.Response):
        if self.cam_to_map_trans is None:
            return res
        
        if req.use_camera:
            cam_channel = self._find_cam_channel(req)

        if req.use_lidar:
            lidar_channel = self._find_lidar_channel(req, cam_channel)
            res.left = lidar_channel[0]
            res.right = lidar_channel[1]
        else:
            res.left = cam_channel[0]
            res.right = cam_channel[1]
        
        return res
    
    def _find_lidar_channel(self, req:Channel.Request, 
        cam_channel:List[Point]):

        channel = [FindChannel.null_point, FindChannel.null_point]

        return channel
    
    def _find_cam_channel(self, req:Channel.Request):

        channel = [FindChannel.null_point, FindChannel.null_point]

        if self.camera_buoys is None:
            return channel

        left_buoys:List[Buoy] = list()
        right_buoys:List[Buoy] = list()
        for buoy in self.camera_buoys.buoys:
            if math.sqrt(buoy.location.x**2 + buoy.location.y**2) > req.max_dist_from_usv:
                continue
            if buoy.color == req.left_color:
                left_buoys.append(buoy)
            elif buoy.color == req.right_color:
                right_buoys.append(buoy)
        
        if len(left_buoys) > 0:
            closest = FindChannel._find_closest_buoy(left_buoys)
            channel[0].x = closest.location.x
            channel[0].y = closest.location.y
            channel[0] = FindChannel._transform_point(self.cam_to_map_trans, channel[0])
        
        if len(right_buoys) > 0:
            closest = FindChannel._find_closest_buoy(right_buoys)
            channel[1].x = closest.location.x
            channel[1].y = closest.location.y
            channel[1] = FindChannel._transform_point(self.cam_to_map_trans, channel[1])
        
        return channel