from geometry_msgs.msg import Point32, PoseStamped

def point32ToPoseStamped(p:Point32):
    ps = PoseStamped()
    ps.pose.position.x = p.x
    ps.pose.position.y = p.y
    ps.pose.position.z = p.z
    return ps