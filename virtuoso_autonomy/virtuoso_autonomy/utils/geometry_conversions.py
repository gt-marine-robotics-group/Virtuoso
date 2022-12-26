from geometry_msgs.msg import Point32, PoseStamped, Quaternion, Point

def point32_to_pose_stamped(p:Point32):
    ps = PoseStamped()
    ps.pose.position.x = p.x
    ps.pose.position.y = p.y
    ps.pose.position.z = p.z
    return ps

def point_to_pose_stamped(p:Point):
    ps = PoseStamped()
    ps.pose.position.x = p.x
    ps.pose.position.y = p.y
    ps.pose.position.z = p.z
    return ps

def xy_and_quaternion_to_pose_stamped(point, orientation:Quaternion):
    ps = PoseStamped()
    ps.pose.position.x = point[0]
    ps.pose.position.y = point[1]
    ps.pose.orientation = orientation
    return ps