import math
from virtuoso_msgs.msg import CostmapPointInfo
from typing import List
from ..utils.get_nodes import get_nodes
from geometry_msgs.msg import Point

class TestGetNodes():

    def __init__(self, width:int, height:int, obstacles:List[int]):

        self.points = []
        
        i = 0
        while i < width * height:
            point = CostmapPointInfo()
            point.x = float(i - (math.trunc(i / width) * width))
            point.y = float(math.trunc(i / width))

            if i in obstacles:
                point.cost = 100
            else:
                point.cost = 0
            
            self.points.append(point)
            
            i += 1
        
    
    def test(self, pos:Point, goal:Point):
        return get_nodes(pos, goal, self.points)


def test1():
    test = TestGetNodes(9, 9, [3])

    pos = Point()
    pos.x = 1.75 # not exactly on a point, should go to node with x = 2
    pos.y = 1.0
    
    goal = Point()
    goal.x = 10.0 # outside of map, should go to node with x = 8
    goal.y = 1.0

    result = test.test(pos, goal)

    # print(str((result[0].pos[0], result[0].pos[1])))
    # print(str((result[1].pos[0], result[1].pos[1])))
    # for node in result[2]:
    #     if node is not None:
    #         print(str((node.pos[0], node.pos[1])))
    #     else:
    #         print(str(None))

def main():
    test1()

if __name__ == '__main__':
    main()