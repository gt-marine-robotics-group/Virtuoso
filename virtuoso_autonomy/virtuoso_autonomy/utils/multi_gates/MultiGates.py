import math
from typing import List
from geometry_msgs.msg import PoseStamped

class MultiGates():

    def find_gates(self, buoys:List[PoseStamped], loc:PoseStamped):

        if len(buoys) < 4:
            return None
        
        buoys.sort(key=lambda b: MultiGates.distance(loc, b))

        buoys = buoys[:4]

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
