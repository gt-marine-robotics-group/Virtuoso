from collections import deque
from typing import Tuple
from rclpy.node import Node
import bisect
from math import sqrt
import numpy as np

class FindDockEntrances:

    def __init__(self):

        self.points:list[Tuple(float,float,float)] = None
        
        self._curr_docks = list()
        self._docks = list(deque(maxlen=20) for _ in range(4))
        self._points_by_dist = list()

        self.node:Node = None # for debugging
    
    def debug(self, msg):
        if self.node is None:
            return
        self.node.get_logger().info(msg)
    
    def find_entrances(self, node:Node=None):

        if self.points is None:
            return
        
        self.node = node

        self._update_docks()

    def _update_docks(self):
        self._curr_docks = list()
        self._update_points_by_dist()
        # self.node.get_logger().info(str(self._points_by_dist))
        if len(self._points_by_dist) < 2:
            return
        self._find_dock_in_front()
        self.node.publish_first_two(self._curr_docks)
        self._find_remaining_docks()
        self.node.publish_current(self._curr_docks)

        self.node.get_logger().info(f'current docks: {self._curr_docks}')
        if len(self._curr_docks) < 4:
            return
        
    
    def _update_points_by_dist(self):
        # ideally we would use a BST instead of sorted list
        # due to O(n) insertion time, but we're likely not working with 
        # a lot of points due point filtering done earlier so no huge
        # performance loss probably
        dists = list()
        dist_to_point = dict()
        self._points_by_dist = list()

        for point in self.points:
            dist = self.distance(point, (0,0))
            bisect.insort(dists, dist)
            dist_to_point[dist] = point

        for dist in dists:
            self._points_by_dist.append(dist_to_point[dist])

    def _find_dock_in_front(self):
        for point in self._points_by_dist:
            if len(self._curr_docks) == 0:
                self._curr_docks.append(point)
                continue
            if (self.distance(self._curr_docks[0], point) < 2
                or abs(self._curr_docks[0][1] - point[1]) < 1):
                continue
            self._curr_docks.append(point)
            return
    
    def _find_remaining_docks(self):
        change = (self._curr_docks[0][0] - self._curr_docks[1][0],
            self._curr_docks[0][1] - self._curr_docks[1][1])
        change_doubled = tuple(2 * c for c in change)
        self.node.get_logger().info(f'change: {change}')
        self.node.get_logger().info(f'curr_docks: {self._curr_docks}')
        possible_points = [np.add(self._curr_docks[0], change),
            np.add(self._curr_docks[0], change_doubled),
            np.subtract(self._curr_docks[1], change), np.subtract(self._curr_docks[1], change_doubled)]
        self.node.publish_possible(possible_points)
        self.node.get_logger().info(f'possible_points: {possible_points}')

        closest_points = list((None, None, None) for _ in possible_points)

        for point in self._points_by_dist:
            for i, p_point in enumerate(possible_points):
                dist = self.distance(point, p_point)
                if closest_points[i][0] is None or dist < closest_points[i][1]:
                    closest_points[i] = (point, dist, i)
        
        closest_points.sort(key=lambda p: p[1])

        self.node.get_logger().info(str(closest_points))

        if closest_points[0][2] < 2:
            self._curr_docks.insert(0, closest_points[0][0])
        else:
            self._curr_docks.append(closest_points[0][0])
        
        if closest_points[1][2] < 2:
            if closest_points[0][2] >= 2 or closest_points[0][2] == 0:
                self._curr_docks.insert(0, closest_points[1][0])
            else:
                self._curr_docks.insert(1, closest_points[1][0])
        else:
            if closest_points[0][2] < 3:
                self._curr_docks.append(closest_points[1][0])
            else:
                self._curr_docks.insert(2, closest_points[1][0])
    
    def distance(self, p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)