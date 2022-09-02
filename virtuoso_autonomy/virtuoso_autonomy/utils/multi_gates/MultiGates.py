import math
from typing import List

from soupsieve import closest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

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

        # points = [
        #     ((buoy_pos.x + 5, buoy_pos.y), False),
        #     ((buoy_pos.x - 5, buoy_pos.y), False),
        #     ((buoy_pos.x, buoy_pos.y + 5), False),
        #     ((buoy_pos.x, buoy_pos.y - 5), False)
        # ]
        # distances = list(MultiGates.distance(MultiGates.xy_to_pose_stamped(p[0]), loc) for p in points)
        # (closest_i, dist) = MultiGates.find_closest_index(distances)

        # points[closest_i][1] = True
        # distances2 = list(d for i, d in enumerate(distances) if i is not closest_i)
        # closest_i, dist = MultiGates.find_closest_index(distances2)
        # closest_i = distances.index(dist)
        # path.poses.append(MultiGates.xy_to_pose_stamped(points[closest_i][0]))
        # points[closest_i][1] = True

        # closest_i = (closest_i + 2) % 4
        # path.poses.append(MultiGates.xy_to_pose_stamped(points[closest_i][0]))
        # points[closest_i][1] = True



        

    def xy_to_pose_stamped(point):
        ps = PoseStamped()
        ps.pose.position.x = point[0]
        ps.pose.position.y = point[1]
        return ps

    def find_closest_index(dists:List[float]):
        min_dist = dists[0] 
        index = 0
        for i, dist in enumerate(dists):
            if dist < min_dist:
                index = i
        return index, min_dist



