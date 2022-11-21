import math
from geometry_msgs.msg import PoseStamped

def distance_pose_stamped(p1:PoseStamped, p2:PoseStamped):
    return math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 + (p1.pose.position.y - p2.pose.position.y)**2)

def same_loc_pose_stamped(p1:PoseStamped, p2:PoseStamped):
    return p1.pose.position.x == p2.pose.position.x and p1.pose.position.y == p2.pose.position.y

def xy_midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)