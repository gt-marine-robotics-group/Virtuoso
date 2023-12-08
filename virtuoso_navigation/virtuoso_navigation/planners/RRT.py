from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from virtuoso_navigation.planners.Planner import Planner
import numpy as np

class RRT(Planner):

    def __init__(self, inflation_layer):
        super().__init__(inflation_layer)

        self._MAX_ITER_COUNT = 1_000_000
        self._step_dist = 0.5
        self._line_collision_check_granularity = 0.1
    
    def distance(x1, y1, x2, y2):
        return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

    def is_occupied(self, x, y):

        # Note that x and y are coming in the map frame but we should not index the costmap
        # with that x and y. It must be transformed to the costmap indices. 

        x_costmap = (x + (self.width_m / 2)) / self.map.info.resolution
        y_costmap = -1 * ((self.height_m / 2) - y) / self.map.info.resolution

        x_index = int(x_costmap)
        y_index = int(y_costmap) - 1

        self.debug(f'index: {(y_index * self.map.info.width) + x_index}')
        if self.map.data[(y_index * self.map.info.width) + x_index] > 0:
            return True

        # self._inflation_layer.map = self.map
        # if self._inflation_layer.in_inflation_layer(x, y):
        #     return True

        return False

    def get_random_loc(self):
        
        if np.random.random_sample() < 0.05:
            return (self.goal.position.x, self.goal.position.y)
        
        x = np.random.uniform(-(self.height_m / 2), self.height_m / 2)
        y = np.random.uniform(-(self.width_m / 2), self.width_m / 2)

        while self.is_occupied(x, y):
            x = np.random.uniform(-(self.height_m / 2), self.height_m / 2)
            y = np.random.uniform(-(self.width_m / 2), self.width_m / 2)
        
        return x, y

    def step_from_to(self, node0, node1):

        # self.debug(f'node0: {node0}')
        # self.debug(f'node1: {node1}')

        if RRT.distance(node0[0], node0[1], node1[0], node1[1]) < self._step_dist:
            return node1
        
        diff = [node1[0] - node0[0], node1[1] - node0[1]]

        diff /= np.sqrt(np.sum(np.square(diff)))

        x = node0[0] + (diff[0] * self._step_dist)
        y = node0[1] + (diff[1] * self._step_dist)

        return x, y
    
    def is_collision_with_obstacles(self, node0: tuple, node1: tuple):
        if self.is_occupied(node1[0], node1[1]): return True

        dist = RRT.distance(node0[0], node0[1], node1[0], node1[1])
        iterations = int(dist / self._line_collision_check_granularity)

        x = node0[0]
        y = node0[1]
        for _ in range(iterations):
            x += (node1[0] - node0[0]) * self._line_collision_check_granularity
            y += (node1[1] - node0[1]) * self._line_collision_check_granularity

            if self.is_occupied(x, y):
                return True
        return False

    def create_path(self, goal: Pose) -> Path:

        self.width_m = self.map.info.width * self.map.info.resolution
        self.height_m = self.map.info.height * self.map.info.resolution

        self.goal = goal

        self.tree = {}
        self.parents = {}

        curr_loc = (self.robot_pose.position.x, self.robot_pose.position.y)

        self.tree[curr_loc] = []
        self.parents[curr_loc] = 0

        goal_node = None

        iteration = 0
        while iteration < self._MAX_ITER_COUNT:
            iteration += 1

            x, y = self.get_random_loc()

            closest_node = None
            closest_dist = 0

            for node in self.tree.keys():
                d = RRT.distance(node[0], node[1], x, y)
                if closest_node is None or d < closest_dist:
                    closest_node = node
                    closest_dist = d
            
            next_node = self.step_from_to(closest_node, (x, y))

            if self.is_collision_with_obstacles(closest_node, next_node):
                self.debug('collision detected')
                continue
            
            self.tree[next_node] = []
            self.tree[closest_node].append(next_node)

            self.parents[next_node] = closest_node

            if RRT.distance(next_node[0], next_node[1], self.goal.position.x, self.goal.position.y) < 0.1:
                goal_node = next_node
                break
        
        if goal_node is None:
            return Path()
        
        path = []

        curr_node = goal_node
        while curr_node != 0:
            path.append(curr_node)
            curr_node = self.parents[curr_node]
        
        path.reverse()

        # add smoothing sometime

        ros_path = Path()
        for node in path:
            p = PoseStamped()
            p.pose.position.x = node[0]
            p.pose.position.y = node[1]
            ros_path.poses.append(p)
        
        return ros_path