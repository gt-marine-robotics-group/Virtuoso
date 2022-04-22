from typing import List
from shapely.geometry import Polygon
from sensor_msgs.msg import PointCloud2
from .pointcloud import create_cloud_xyz32, read_points
from rclpy.client import Client
from robot_localization.srv import FromLL
from tf2_ros.buffer import Buffer
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PointStamped
from rclpy.time import Time
from .geometry_msgs import do_transform_point

class ShoreFilter():

    vrx_border_mappoints = None
    vrx_shore:Polygon = None
    geo_to_map_count = 0
    
    def create_shore(fromLL_cli:Client, tf_buffer:Buffer):

        if (ShoreFilter.vrx_border_mappoints is not None):
            ShoreFilter.convert_to_lidar(tf_buffer)
            return

        vrx_border_points = [
            # (lat, long)
            (-33.72279754, 150.6739663),
            (-33.7228804, 150.6747816),
            (-33.7225852, 150.6749084),
            (-33.7226768, 150.6752924),
            (-33.7229284, 150.6751755),
            (-33.7229839, 150.67387875),
            (-33.7225977, 150.6733497),
            (-33.7220645, 150.6736035),
            (-33.7220424, 150.6738909),
            (-33.72272601, 150.6737193)
        ]

        vrx_border_geopoints = list()
        for p in vrx_border_points:
            geoPoint = GeoPoint()
            geoPoint.latitude = p[0]
            geoPoint.longitude = p[1]
            vrx_border_geopoints.append(geoPoint)

        ShoreFilter.vrx_border_mappoints:List[Point] = list()

        def ll_callback(future):
            point:Point = future.result().map_point
            ShoreFilter.vrx_border_mappoints.append(point)
            ShoreFilter.geo_to_map_count += 1
            if (ShoreFilter.geo_to_map_count < len(vrx_border_geopoints)):
                req = FromLL.Request()
                req.ll_point = vrx_border_geopoints[ShoreFilter.geo_to_map_count]
                ll = fromLL_cli.call_async(req)
                ll.add_done_callback(ll_callback)
            else:
                ShoreFilter.convert_to_lidar(tf_buffer)

        if (ShoreFilter.geo_to_map_count >= len(vrx_border_geopoints)):
            return
        
        req = FromLL.Request()
        req.ll_point = vrx_border_geopoints[ShoreFilter.geo_to_map_count]
        ll = fromLL_cli.call_async(req)
        ll.add_done_callback(ll_callback)

    def convert_to_lidar(tf_buffer:Buffer):

        trans = None
        try:
            trans = tf_buffer.lookup_transform('wamv/lidar_wamv_link', 'map', Time())
        except:
            return
        
        vrx_shore_list = list()

        for p in ShoreFilter.vrx_border_mappoints:
            pStamped = PointStamped()
            pStamped.point = p
            pStamped.header.frame_id = 'map'
            transPoint = do_transform_point(pStamped, trans)
            vrx_shore_list.append((transPoint.point.x, transPoint.point.y))

        ShoreFilter.vrx_shore = Polygon(vrx_shore_list) 

    def filter_points(points:PointCloud2):

        filtered_points = []



