from geometry_msgs.msg import TransformStamped, Point, PointStamped
from virtuoso_msgs.msg import BoundingBoxArray, BoundingBox, BuoyArray, Buoy
from virtuoso_msgs.srv import Channel
from nav_msgs.msg import Odometry
from typing import List
import math
from ..utils.geometry_msgs import do_transform_point

class FindChannel: 

    null_point = Point(x=0.0,y=0.0,z=0.0)
    
    def __init__(self):

        self.node = None

        self.cam_to_map_trans:TransformStamped = None
        self.lidar_to_map_trans:TransformStamped = None
        
        self.lidar_buoys:BoundingBoxArray = None
        self.camera_buoys:BuoyArray = None

        self.odom:Odometry = None

        # When count becomes greater than some number, just fall back to 
        # using LIDAR only if that is an option
        self.iteration_count = 0

    def _find_closest_buoy(buoys:List[Buoy]):
        return min(buoys, key=lambda b: math.sqrt(b.location.x**2 + b.location.y**2))
    
    def _transform_point(trans:TransformStamped, point:Point):
        ps = PointStamped(point=point)

        trans_ps = do_transform_point(ps, trans)

        return trans_ps.point
    
    def _distance(p1:Point, p2:Point):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    
    def _box_to_point(box:BoundingBox):
        return Point(x=box.centroid.x, y=box.centroid.y, z=box.centroid.z)
    
    def reset(self):
        self.lidar_buoys = None
        self.camera_buoys = None
        self.iteration_count = 0
    
    def execute(self, req:Channel.Request, res:Channel.Response):
        if req.use_lidar and self.lidar_to_map_trans is None:
            return res

        if req.use_camera and self.cam_to_map_trans is None:
            return res
        
        self.iteration_count += 1
        
        cam_channel = None
        if req.use_camera and self.iteration_count < 9:
            cam_channel = self._find_cam_channel(req)
            if cam_channel[0] == FindChannel.null_point:
                if cam_channel[1] == FindChannel.null_point:
                    return res
                res.left = cam_channel[0]
                res.right = cam_channel[1]
                return res
            elif cam_channel[1] == FindChannel.null_point:
                res.left = cam_channel[0]
                res.right = cam_channel[1]
                return res
        
        if req.use_lidar or self.iteration_count > 8:
            lidar_channel = self._find_lidar_channel(req, cam_channel)
            res.left = lidar_channel[0]
            res.right = lidar_channel[1]
        elif req.use_camera:
            res.left = cam_channel[0]
            res.right = cam_channel[1]
        
        return res
    
    def _find_lidar_channel(self, req:Channel.Request, 
        cam_channel:List[Point]):

        channel = [Point(x=0.0,y=0.0,z=0.0), Point(x=0.0,y=0.0,z=0.0)]

        if self.lidar_buoys is None or self.odom is None:
            return cam_channel or channel
        
        lidar_points = list(
            FindChannel._transform_point(
                self.lidar_to_map_trans,
                FindChannel._box_to_point(box)
            ) for box in self.lidar_buoys.boxes
        )
        
        left_dists = list()
        right_dists = list()
        usv_dists = list()

        for buoy in lidar_points:
            if cam_channel is not None:
                left_dists.append(FindChannel._distance(buoy, cam_channel[0]))
                right_dists.append(FindChannel._distance(buoy, cam_channel[1]))
            usv_dists.append(FindChannel._distance(buoy, 
                self.odom.pose.pose.position))
        
        if len(usv_dists) < 2:
            if len(usv_dists) == 1:
                channel[0] = lidar_points[0]
            return cam_channel or channel

        if cam_channel is not None:
            min_left_dists_index = min(range(len(left_dists)), key=left_dists.__getitem__)
            min_right_dists_index = min(range(len(right_dists)), key=right_dists.__getitem__)
            if min_left_dists_index == min_right_dists_index:
                return channel
            channel[0] = lidar_points[min_left_dists_index]
            channel[1] = lidar_points[min_right_dists_index]
        else:
            mins = [[math.inf, 0], [math.inf, 0]]
            for i, dist in enumerate(usv_dists):
                if dist < mins[0][0]:
                    mins[1] = mins[0]
                    mins[0] = [dist, i]
                elif dist < mins[1][0]:
                    mins[1] = [dist, i]
            channel[0] = lidar_points[mins[0][1]]
            channel[1] = lidar_points[mins[1][1]]

        return channel
    
    def _find_cam_channel(self, req:Channel.Request):

        channel = [Point(x=0.0,y=0.0,z=0.0), Point(x=0.0,y=0.0,z=0.0)]

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
