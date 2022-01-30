from geometry_msgs.msg import Point
import math
from ..utils.Node import AstarNode
from ..utils.astar import astar

# For reading grids drawn above tests:
# 0 = space is unoccupied
# x = obstacle
# s = start
# e = end

class TestAstar():

    def __init__(self, start_i, end_i, width, height, obstacles):
        self.width = width
        self.height = height
        self.nodes = []        

        for i in range(width * height):
            if i in obstacles: 
                self.nodes.append(None)
                continue
            pos = (float(i % width), float(math.trunc(i / width)))
            node = AstarNode(pos, i, None)
            self.nodes.append(node)
            
            if i == start_i: self.start_node = node
            if i == end_i: self.end_node = node
        
    def test(self):
        path = astar(self.nodes, self.start_node, self.end_node, (self.width, self.height))
        arr = []
        for point in path:
            arr.append((point[0], point[1]))
        return arr


# 0 0 0 0 s
# 0 0 0 0 x
# 0 0 0 0 e
# 0 0 0 0 0 
# 0 0 0 0 0
def test1():
    test = TestAstar(4, 14, 5, 5, [9])
    path = test.test()
    solution = [(4.0, 0.0), (3.0, 0.0), (3.0, 1.0), (3.0, 2.0), (4.0, 2.0)]

    assert path == solution

# 0 0 0 0 0 
# 0 0 0 0 0 
# 0 0 s x 0
# 0 0 x e 0
# 0 0 0 0 0
def test2():
    test = TestAstar(12, 18, 5, 5, [13, 17])
    path = test.test()
    solution1 = [(2.0, 2.0), (2.0, 1.0), (3.0, 1.0), (4.0, 1.0), (4.0, 2.0), (4.0, 3.0), (3.0, 3.0)]
    solution2 = [(2.0, 2.0), (1.0, 2.0), (1.0, 3.0), (1.0, 4.0), (2.0, 4.0), (3.0, 4.0), (3.0, 3.0)]

    assert path == solution1 or path == solution2

def main():
    test1()
    test2()

if __name__ == '__main__':
    main()