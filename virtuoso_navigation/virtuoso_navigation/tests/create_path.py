from ..utils.create_path import create_path
from ..utils.Node import AstarNode

def main():

    nodes = [(float(i), float(i))for i in range(9)]

    path = create_path(nodes)

    # for pos in path.poses:
    #     print(str(pos.pose.position))

if __name__ == '__main__':
    main()