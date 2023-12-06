from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from virtuoso_navigation.planners.Planner import Planner
import numpy as np

class RRT(Planner):

    def __init__(self):
        self.extra_after_goal = 10
        self.extra_before_robot = 3

        self.expand_size = 10

    def is_occupied(self, x, y):

        # add origin of self.map to the robot_pose x and y
        # subtract extra_before_robot
        # add the x and y positions passed in

        #nope
        # should first get four x,y grid bounds based on extra_dist in self.create_path
        # then the input x and y to those
        pass

    def get_random_loc(self):
        
        if np.random.random_sample() < 0.05:
            return (self.goal.position.x, self.goal.position.y)
        
        x = np.random.random_sample() * self.search_x_dist
        y = np.random.random_sample() * self.search_y_dist

        # check if occupied


    def create_path(self, goal: Pose) -> Path:

        self.goal = goal

        extra_dist = self.extra_after_goal + self.extra_before_robot

        self.search_x_dist = abs(goal.position.x - self.robot_pose.position.x) + extra_dist
        self.search_y_dist = abs(goal.position.y - self.robot_pose.position.y) + extra_dist

        self.iteration = 0

        self.tree = {}
        self.nodes = set()

        curr_loc = (self.robot_pose.x, self.robot_pose.y)

        self.nodes.add(curr_loc)
        self.tree[curr_loc] = {}