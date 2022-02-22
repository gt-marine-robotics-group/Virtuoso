from pdb import post_mortem
from geographic_msgs.msg import GeoPoseStamped
from tf2_ros import Buffer
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from rclpy.time import Time
from geodesy import utm

def calc_pose(geo_pose:GeoPoseStamped, tf_buffer:Buffer):

    geo_point = geo_pose.pose.position
    lat = geo_point.latitude
    lon = geo_point.longitude

    goal_point = utm.fromLatLong(lat, lon)

    point_utm_frame = goal_point.toPoint()

    try:
        trans:TransformStamped = tf_buffer.lookup_transform('odom', 'utm', Time())

        # Calculate the pose in the map frame 
        # pose_stamped = PoseStamped()
        # pose =  Pose()
        # point = Point()
        # point.x = point_utm_frame.x + trans.transform.translation.x
        # point.y = point_utm_frame.y + trans.transform.translation.y
        # pose.position = point
        # pose.orientation = geo_pose.pose.orientation
        # pose_stamped.pose = pose

        starting_point = Point()
        starting_point.x = 284479.920313837
        starting_point.y = -3733847.302744045

        pose_stamped = PoseStamped()
        pose = Pose()
        point = Point()
        point.x = point_utm_frame.x - starting_point.x
        point.y = point_utm_frame.y - starting_point.y
        pose.position = point
        pose_stamped.pose = pose


        return pose_stamped 

    except Exception as e:
        return None