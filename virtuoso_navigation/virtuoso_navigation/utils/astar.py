import math
from geometry_msgs.msg import Point
from .Node import AstarNode


def pythag_dist(p1:Point, p2:Point):
    return (p1.x - p2.x)**2 + (p1.y - p2.y)**2

# calc distance from starting position to point
def calc_dist(p1:Point, p2:Point):
    return math.sqrt(pythag_dist(p1, p2))

def pythag_dist_tuples(p1, p2):
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def calc_dist_tuples(p1, p2):
    return math.sqrt(pythag_dist_tuples(p1, p2))
        
def find_index_w_least_f(nodes):
    lowest_f = math.inf 
    index = 0
    for i, node in enumerate(nodes):
        if node.get_f() < lowest_f:
            lowest_f = node.get_f()
            index = i
    return index

def astar(nodes:list, start_node:AstarNode, end_node:AstarNode, dimensions):
        width = dimensions[0]
        height = dimensions[1]

        open_list = [start_node]
        closed_list = []

        while len(open_list) > 0:

            least_f_index = find_index_w_least_f(open_list)
            current_node = open_list[least_f_index]
            open_list.pop(least_f_index)
            closed_list.append(current_node)

            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.pos)
                    current = current.parent
                
                finished_path = path[::-1] # reverse
                return finished_path
            
            children = []

            i = current_node.index

            north_node = nodes[i - width] if i - width >= 0 and nodes[i - width] is not None else None
            south_node = nodes[i + width] if i + width < width * height and nodes[i + width] is not None else None
            east_node = nodes[i + 1] if i % width != width - 1 and nodes[i + 1] is not None else None
            west_node = nodes[i - 1] if i % width != 0 and nodes[i - 1] is not None else None

            if north_node is not None: children.append(AstarNode(north_node.pos, i - width, current_node))
            if south_node is not None: children.append(AstarNode(south_node.pos, i + width, current_node))
            if east_node is not None: children.append(AstarNode(east_node.pos, i + 1, current_node))
            if west_node is not None: children.append(AstarNode(west_node.pos, i - 1, current_node))
            

            for child in children:
                
                if child in closed_list: continue
                
                # distance from child to start
                child.g = current_node.g + calc_dist_tuples(current_node.pos, child.pos) 

                # distance from child to end position
                child.h = pythag_dist_tuples(child.pos, end_node.pos)  

                # Child already in open list
                already_opened = False
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        already_opened = True
                        break 
                if already_opened: continue

                open_list.append(child)


