from collections import deque
from typing import Tuple
from rclpy.node import Node
import bisect
from math import sqrt

class FindDockEntrances:

    def __init__(self):

        self.points:list[Tuple(float,float,float)] = None
        
        self._docks = list(deque(maxlen=20) for _ in range(4))
        self._points_by_dist = list()

        self.node:Node = None # for debugging
    
    def find_entrances(self, node:Node=None):

        if self.points is None:
            return
        
        self.node = node

        self._update_docks()

    def _update_docks(self):
        self._update_points_by_dist()
        self.node.get_logger().info(str(self._points_by_dist))
    
    def _update_points_by_dist(self):
        # ideally we would use a BST instead of sorted list
        # due to O(n) insertion time, but we're likely not working with 
        # a lot of points due point filtering done earlier so no huge
        # performance loss probably
        dists = list()
        dist_to_point = dict()
        self._points_by_dist = list()

        for point in self.points:
            dist = self.distance(point)
            # bisect.insort(self._points_by_dist, (point, dist),
            #     key=lambda p: p[1])
            bisect.insort(dists, dist)
            dist_to_point[dist] = point

        for dist in dists:
            self._points_by_dist.append((dist_to_point[dist], dist))

    def _find_dock_in_front(self):
        pass
    
    def distance(self, point):
        return sqrt(point[0]**2 + point[1]**2)