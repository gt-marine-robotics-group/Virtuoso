from .Node import AstarNode
from typing import List, Tuple
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

def create_path(positions:List[Tuple[float, float]]):

    poses = []

    for i, position in enumerate(positions):
        pose_stamped = PoseStamped()

        pose = Pose()

        pose.position.x = position[0]
        pose.position.y = position[1]

        if i + 1 == len(positions): 
            pose.orientation = poses[i - 1].pose.orientation
            pose_stamped.pose = pose
            poses.append(pose_stamped)
            break

        # TODO
        # need to figure out how to calculate orientation at new point
        # based on angle between point and previous point
        pose.orientation.w = 1.0
        pose.orientation.z = 0.0
        pose.orientation.y = 0.0
        pose.orientation.x = 0.0

        pose_stamped.pose = pose
        poses.append(pose_stamped)

    path = Path()
    path.poses = poses

    return path
