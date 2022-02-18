from .Buoy import Buoy
from typing import List 
from tf2_ros.buffer import Buffer 
from rclpy.time import Time, Duration
import tf2_py
from geometry_msgs.msg import TransformStamped
from geodesy.utm import UTMPoint, gridZone
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

def transform_buoys(buoys:List[Buoy], tf_buffer:Buffer, gps_pos:NavSatFix):

    trans = None
    try:
        trans = tf_buffer.lookup_transform('utm', 'wamv/lidar_wamv_link', Time())
    except Exception as e:
        return None 
    
    for buoy in buoys:
        buoy.geo_msg = create_geo_msg(buoy, trans, gps_pos)
    
    return buoys

def create_geo_msg(buoy:Buoy, trans:TransformStamped, gps_pos:NavSatFix):

    easting = buoy.centroid.x + trans.transform.translation.x
    northing = buoy.centroid.y + trans.transform.translation.y
    altitude = buoy.centroid.z + trans.transform.translation.z

    utm_point =  UTMPoint(easting=easting, northing=northing, altitude=altitude)
    zone, band = gridZone(gps_pos.latitude, gps_pos.longitude)
    utm_point.zone = zone
    utm_point.band = band

    geo_point:GeoPoint = utm_point.toMsg()

    geo_pose_stamped = GeoPoseStamped()
    
    geo_pose = GeoPose()
    geo_pose.position = geo_point
    geo_pose_stamped.pose = geo_pose

    header = Header()
    header.frame_id = buoy.code
    geo_pose_stamped.header = header

    return geo_pose_stamped
