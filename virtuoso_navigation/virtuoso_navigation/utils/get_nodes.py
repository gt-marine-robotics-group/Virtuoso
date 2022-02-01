from typing import Tuple
from typing import List
from geometry_msgs.msg import Point
from .Node import AstarNode
import math
from .astar import pythag_dist
from virtuoso_msgs.msg import CostmapPointInfo

def is_valid_node(point, g):
    if point.cost > 99: return False

    # if cost is unkown but it is next to the boat, assume its an obstacle
    if point.cost == -1 and g < 3: return False

    return True

def get_nodes(pos:Point, goal:Point, points:List[CostmapPointInfo]) -> Tuple[AstarNode, AstarNode, List[AstarNode]]:
    nodes = []

    closest_dist_to_pos = math.inf 
    start_node = None

    closest_dist_to_end = math.inf 
    end_node = None

    for i, point in enumerate(points):

        # distance from point to starting position
        # dont need to take sqrt since not doing it for all
        g = pythag_dist(pos, point) 

        if not is_valid_node(point, g):
            nodes.append(None)
            continue 

        node = AstarNode((point.x, point.y), i, None)

        # dont really need to take sqrt if not doing it for all of them
        h = pythag_dist(point, goal)

        if h < closest_dist_to_end:
            closest_dist_to_end = h 
            end_node = node

        if g < closest_dist_to_pos:
            closest_dist_to_pos = g
            start_node = node

        nodes.append(node)

    return (start_node, end_node, nodes)
