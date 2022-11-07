from typing import Tuple


class LoopPoint:

    def __init__(self, change:str, point:Tuple[float, float], dist:float):
        self.change = change
        self.xy = point
        self.x = point[0]
        self.y = point[1]
        self.dist = dist
        self.used = False