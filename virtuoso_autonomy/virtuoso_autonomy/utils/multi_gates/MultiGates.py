import math
from typing import List
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from .LoopPoint import LoopPoint

class MultiGates():

    first_buoys = None
    buoy_range = 3.0

    def find_gates(self, buoys:List[PoseStamped], loc:PoseStamped):

        if len(buoys) < 4:
            return None
        
        buoys.sort(key=lambda b: MultiGates.distance(loc, b))

        buoys = buoys[:4]

        self.first_buoys = buoys

        farthest_buoy = buoys[3]

        farthest_pair = (farthest_buoy, MultiGates.find_closest_buoy(buoys[:3], farthest_buoy))

        remaining_buoys = list(filter(lambda b: not MultiGates.same_loc(farthest_pair[1], b), buoys[:3]))

        middle_pair = (farthest_pair[1], MultiGates.find_closest_buoy(remaining_buoys, farthest_pair[1]))

        last_buoy = list(filter(lambda b: not MultiGates.same_loc(middle_pair[1], b), remaining_buoys))

        closest_pair = (middle_pair[1], last_buoy[0])

        return (
            closest_pair,
            middle_pair,
            farthest_pair
        )

    def find_closest_buoy(buoys:List[PoseStamped], loc:PoseStamped):
        min_dist = MultiGates.distance(loc, buoys[0])
        min_buoy = buoys[0]
        for i, buoy in enumerate(buoys):
            if i == 0: continue
            dist = MultiGates.distance(loc, buoy)
            if dist < min_dist:
                min_dist = dist
                min_buoy = buoy
        
        return min_buoy

    def distance(p1:PoseStamped, p2:PoseStamped):
        return math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 + (p1.pose.position.y - p2.pose.position.y)**2)

    def same_loc(p1:PoseStamped, p2:PoseStamped):
        return p1.pose.position.x == p2.pose.position.x and p1.pose.position.y == p2.pose.position.y

    def find_closest_gate_to_robot(self, loc:PoseStamped, mids:List[PoseStamped]):
        min_dist = MultiGates.distance(loc, mids[0])
        min_gate = mids[0]
        for i, mid in enumerate(mids):
            if i == 0: continue
            dist = MultiGates.distance(loc, mid)
            if dist < min_dist:
                min_dist = dist
                min_gate = mid
        
        return min_gate
    
    def find_looping_buoy(self, buoys:List[PoseStamped], loc:PoseStamped):

        possible_buoys = []

        for buoy in buoys:
            duplicate = False
            for gate_buoy in self.first_buoys:
                if MultiGates.distance(gate_buoy, buoy) < MultiGates.buoy_range:
                    duplicate = True
                    continue
            if not duplicate:
                possible_buoys.append(buoy)

        if len(possible_buoys) == 0:
            return None
        
        possible_buoys.sort(key=lambda b: MultiGates.distance(loc, b))

        return possible_buoys[0]
    
    def find_path_around_buoy(self, buoy:PoseStamped, loc:PoseStamped):

        path = Path()

        buoy_pos = buoy.pose.position

        points = [
            (buoy_pos.x + 5, buoy_pos.y),
            (buoy_pos.x - 5, buoy_pos.y),
            (buoy_pos.x, buoy_pos.y + 5),
            (buoy_pos.x, buoy_pos.y - 5)
        ]

        points = list(
            LoopPoint('x' if i < 2 else 'y', p, MultiGates.distance(MultiGates.xy_to_pose_stamped(p), loc))
            for i, p in enumerate(points)
        )

        closest_i = MultiGates.find_closest_index(points)
        # path.poses.append(MultiGates.xy_to_pose_stamped(points[closest_i].xy))
        points[closest_i].used = True

        closest_i = MultiGates.find_closest_index(points)
        path.poses.append(MultiGates.xy_to_pose_stamped(points[closest_i].xy))
        points[closest_i].used = True

        for point in points:
            if point.used:
                continue
            if point.change is not points[closest_i].change:
                path.poses.append(MultiGates.xy_to_pose_stamped(point.xy))
                point.used = True
                break
        
        for point in points:
            if point.used:
                continue
            path.poses.append(MultiGates.xy_to_pose_stamped(point.xy))
            break

        # path.poses.insert(0, path.poses[len(path.poses) - 1])

        # We can later remove this and have the robot choose a random gate to go through
        path.poses.append(loc)

        return path

    def xy_to_pose_stamped(point):
        ps = PoseStamped()
        ps.pose.position.x = point[0]
        ps.pose.position.y = point[1]
        return ps

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



