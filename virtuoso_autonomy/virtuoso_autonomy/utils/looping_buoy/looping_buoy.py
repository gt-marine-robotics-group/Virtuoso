from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_transformations
from .loop_point import LoopPoint
from ..math import distance_pose_stamped
from ..geometry_conversions import xy_and_quaternion_to_pose_stamped
from typing import List
import math

class LoopingBuoy:

    def _find_closest_index(points:List[LoopPoint]):
        min_dist = None
        index = -1
        for i, point in enumerate(points):
            if point.used:
                continue
            if index == -1:
                min_dist = point.dist
                index = i
            elif point.dist < min_dist:
                min_dist = point.dist
                index = i
        return index

    def find_path_around_buoy(robot_pose:PoseStamped, buoy:PoseStamped, looping_radius=5.0):

        path = Path()

        rq = robot_pose.pose.orientation
        r_euler = tf_transformations.euler_from_quaternion([rq.x, rq.y, rq.z, rq.w])
        rrq = tf_transformations.quaternion_from_euler(r_euler[0], r_euler[1], r_euler[2] + math.pi)
        rq_reverse = Quaternion()
        rq_reverse.x = rrq[0]
        rq_reverse.y = rrq[1]
        rq_reverse.z = rrq[2]
        rq_reverse.w = rrq[3]

        buoy_pos = buoy.pose.position

        points = [
            (buoy_pos.x + looping_radius, buoy_pos.y),
            (buoy_pos.x - looping_radius, buoy_pos.y),
            (buoy_pos.x, buoy_pos.y + looping_radius),
            (buoy_pos.x, buoy_pos.y - looping_radius)
        ]

        points = list(
            LoopPoint('x' if i < 2 else 'y', p, 
                distance_pose_stamped(xy_and_quaternion_to_pose_stamped(p, rq), robot_pose)
            )
            for i, p in enumerate(points)
        )

        closest_i = LoopingBuoy._find_closest_index(points)
        path.poses.append(xy_and_quaternion_to_pose_stamped(points[closest_i].xy, rq))
        points[closest_i].used = True

        for i, point in enumerate(points):
            if point.used:
                continue
            if point.change is not points[closest_i].change:
                closest_i = i
                path.poses.append(xy_and_quaternion_to_pose_stamped(point.xy, rq))
                point.used = True
                break

        for i, point in enumerate(points):
            if point.used:
                continue
            if point.change is not points[closest_i].change:
                closest_i = i
                path.poses.append(xy_and_quaternion_to_pose_stamped(point.xy, rq_reverse))
                point.used = True
                break
        
        for point in points:
            if point.used:
                continue
            path.poses.append(xy_and_quaternion_to_pose_stamped(point.xy, rq_reverse))
            break

        final = PoseStamped()
        final.pose.position = robot_pose.pose.position
        final.pose.orientation = rq_reverse
        path.poses.append(final)

        return path
