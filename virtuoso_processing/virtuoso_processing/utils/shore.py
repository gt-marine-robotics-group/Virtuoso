from typing import List
from shapely.geometry import Polygon, Point as ShapelyPoint
from shapely.affinity import scale
from sensor_msgs.msg import PointCloud2
from .pointcloud import create_cloud_xyz32, read_points
from rclpy.client import Client
from robot_localization.srv import FromLL
from tf2_ros.buffer import Buffer
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PointStamped
from rclpy.time import Time
from .geometry_msgs import do_transform_point
import matplotlib.pyplot as plt

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
            (-33.7227681, 150.673908), # where the wamv starts
            (-33.7225852, 150.6749084), 
            (-33.7226768, 150.6752924), 
            (-33.7229284, 150.6751755),
            (-33.7236431, 150.6737446),
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
            if (point.x == 0.0):
                ShoreFilter.geo_to_map_count = -1
                return
            ShoreFilter.vrx_border_mappoints.append(point)
            ShoreFilter.geo_to_map_count += 1
            if (ShoreFilter.geo_to_map_count < len(vrx_border_geopoints)):
                req = FromLL.Request()
                req.ll_point = vrx_border_geopoints[ShoreFilter.geo_to_map_count]
                ll = fromLL_cli.call_async(req)
                ll.add_done_callback(ll_callback)
            else:
                ShoreFilter.convert_to_lidar(tf_buffer)
        
        if (ShoreFilter.geo_to_map_count == -1):
            ShoreFilter.vrx_border_mappoints = None
            ShoreFilter.geo_to_map_count = 0
            return

        if (ShoreFilter.geo_to_map_count >= len(vrx_border_geopoints)):
            return
        
        req = FromLL.Request()
        req.ll_point = vrx_border_geopoints[ShoreFilter.geo_to_map_count]
        ll = fromLL_cli.call_async(req)
        ll.add_done_callback(ll_callback)

    def convert_to_lidar(tf_buffer:Buffer):

        if ShoreFilter.vrx_shore is not None:
            return

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
            # vrx_shore_list.append((p.x, p.y))

        # ShoreFilter.vrx_shore = scale(Polygon(vrx_shore_list), xfact=1.5, yfact=1.5, zfact=1.5, origin='center')
        ShoreFilter.vrx_shore = Polygon(vrx_shore_list)
        # ShoreFilter.vrx_shore = [
        #     Polygon([vrx_shore_list[0], vrx_shore_list[9], vrx_shore_list[5]]),
        #     Polygon([vrx_shore_list[7], vrx_shore_list[8], vrx_shore_list[9], vrx_shore_list[6]]),
        #     Polygon([vrx_shore_list[0], vrx_shore_list[1], vrx_shore_list[5]]),
        #     Polygon([vrx_shore_list[2], vrx_shore_list[3], vrx_shore_list[4], vrx_shore_list[1]])
        # ]

    def filter_points(points:PointCloud2):

        if not ShoreFilter.vrx_shore:
            return

        filtered_points = []

        for i, point in enumerate(read_points(points, field_names=('x', 'y', 'z'))):
            # if (ShoreFilter.vrx_shore.contains(ShapelyPoint(point[0], point[1]))):
            # for poly in ShoreFilter.vrx_shore:
            #     if (poly.contains(ShapelyPoint(point[0], point[1]))):
            #         filtered_points.append([float("NaN") for _ in range(3)])
            #         break
            # else:
                # filtered_points.append([point[j] for j in range(3)])
            if (ShapelyPoint(point[0], point[1]).within(ShoreFilter.vrx_shore)):
                filtered_points.append([float("NaN") for _ in range(3)])
            else:
                filtered_points.append([point[j] for j in range(3)])
        
        filtered_cloud = create_cloud_xyz32(points.header, filtered_points)

        return filtered_cloud

        



