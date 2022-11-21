from typing import List
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
from ...utils.math import xy_midpoint

class Docking:

    def __init__(self, target_dock_color, dock_approach_dist):

        self._target_dock_color = target_dock_color
        self._dock_approach_dist = dock_approach_dist

        self.robot_pose:PoseStamped = None

        self._color_docks = dict() # map of color => index
        self.entrances = list() # list of 4 points (x, y)
        self.ahead_entrance = list() # 2 points
    
    def find_approach_point(self):
        if len(self.ahead_entrance) < 2:
            return None
        
        mid = ((self.ahead_entrance[0][0] + self.ahead_entrance[1][0]) / 2, 
            (self.ahead_entrance[0][1] + self.ahead_entrance[1][1]) / 2)
        
        return Point(x=mid[0]-self._dock_approach_dist, y=mid[1])
    
    def populate_color_docks(self, data:List[int]):
        if 1 in data[3:6]:
            return

        offset_to_color = dict()
        offset_to_color[data[0]] = 'red'
        offset_to_color[data[1]] = 'green'
        offset_to_color[data[2]] = 'blue'

        offsets = list(data[0:3])
        offsets.sort()

        for i, offset in enumerate(offsets):
            self._color_docks[offset_to_color[offset]] = i
    
    def find_y_translate_point(self):
        if not self._color_docks:
            return None
        if len(self.ahead_entrance) < 2:
            return None
        
        p1 = None
        p2 = None
        change = (
            self.ahead_entrance[1][0] - self.ahead_entrance[0][0],
            self.ahead_entrance[1][1] - self.ahead_entrance[0][1]
        )
        index = self._color_docks[self._target_dock_color] 
        if index == 0:
            p1 = self.ahead_entrance[0]
            p2 = np.subtract(p1, change)
        elif index == 1:
            p1 = self.ahead_entrance[0]
            p2 = self.ahead_entrance[1]
        else:
            p1 = self.ahead_entrance[1]
            p2 = np.add(p1, change)
        
        mid = xy_midpoint(p1, p2)

        return Point(x=mid[0]-self._dock_approach_dist, y=mid[1])

    def find_entrance_point(self):

        if len(self.ahead_entrance) < 2:
            return None

        mid = ((self.ahead_entrance[0][0] + self.ahead_entrance[1][0]) / 2, 
            (self.ahead_entrance[0][1] + self.ahead_entrance[1][1]) / 2)
        
        return Point(x=mid[0], y=mid[1])

