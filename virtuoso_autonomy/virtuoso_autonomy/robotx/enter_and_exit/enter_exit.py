from typing import List
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import tf_transformations
from ...utils.math import distance_pose_stamped, same_loc_pose_stamped
from ...utils.geometry_conversions import xy_and_quaternion_to_pose_stamped
import math
from .loop_point import LoopPoint

class EnterExit:

    first_buoys = None
    buoy_range = 3.0

    def __init__(self, node=None):

        self.node:Node = node

        self.robot_pose:PoseStamped = None
    
    def _debug(self, msg):
        if self.node is None:
            return
        self.node.get_logger().info(msg)

    def find_closest_buoy(buoys:List[PoseStamped], loc:PoseStamped):
        min_dist = distance_pose_stamped(loc, buoys[0])
        min_buoy = buoys[0]
        for i, buoy in enumerate(buoys):
            if i == 0: continue
            dist = distance_pose_stamped(loc, buoy)
            if dist < min_dist:
                min_dist = dist
                min_buoy = buoy
        
        return min_buoy
    
    def find_closest_index(points:List[LoopPoint]):
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
    
    def find_gates(self, buoys:List[PoseStamped]):
        
        if len(buoys) < 4:
            return None
        
        buoys.sort(key=lambda b: distance_pose_stamped(self.robot_pose, b))

        buoys = buoys[:4]

        self.first_buoys = buoys

        farthest_buoy = buoys[3]

        farthest_pair = (farthest_buoy, 
            EnterExit.find_closest_buoy(buoys[:3], farthest_buoy))

        remaining_buoys = list(
            filter(lambda b: not same_loc_pose_stamped(farthest_pair[1], b), buoys[:3])
        )

        middle_pair = (farthest_pair[1], 
            EnterExit.find_closest_buoy(remaining_buoys, farthest_pair[1]))

        last_buoy = list(
            filter(lambda b: not same_loc_pose_stamped(middle_pair[1], b), remaining_buoys)
        )

        closest_pair = (middle_pair[1], last_buoy[0])

        return (
            closest_pair,
            middle_pair,
            farthest_pair
        )
    
    def find_looping_buoy(self, buoys:List[PoseStamped]):

        possible_buoys = []

        for buoy in buoys:
            duplicate = False
            for gate_buoy in self.first_buoys:
                if distance_pose_stamped(gate_buoy, buoy) < EnterExit.buoy_range:
                    duplicate = True
                    continue
            if not duplicate:
                possible_buoys.append(buoy)

        if len(possible_buoys) == 0:
            return None
        
        possible_buoys.sort(key=lambda b: distance_pose_stamped(self.robot_pose, b))

        return possible_buoys[0]

    def find_path_around_buoy(self, buoy:PoseStamped):

        path = Path()

        rq = self.robot_pose.pose.orientation
        r_euler = tf_transformations.euler_from_quaternion([rq.x, rq.y, rq.z, rq.w])
        rrq = tf_transformations.quaternion_from_euler(r_euler[0], r_euler[1], r_euler[2] + math.pi)
        rq_reverse = Quaternion()
        rq_reverse.x = rrq[0]
        rq_reverse.y = rrq[1]
        rq_reverse.z = rrq[2]
        rq_reverse.w = rrq[3]

        buoy_pos = buoy.pose.position

        points = [
            (buoy_pos.x + 5, buoy_pos.y),
            (buoy_pos.x - 5, buoy_pos.y),
            (buoy_pos.x, buoy_pos.y + 5),
            (buoy_pos.x, buoy_pos.y - 5)
        ]

        points = list(
            LoopPoint('x' if i < 2 else 'y', p, 
                distance_pose_stamped(xy_and_quaternion_to_pose_stamped(p, rq), self.robot_pose)
            )
            for i, p in enumerate(points)
        )

        closest_i = EnterExit.find_closest_index(points)
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
        final.pose.position = self.robot_pose.pose.position
        final.pose.orientation = rq_reverse
        path.poses.append(final)

        return path