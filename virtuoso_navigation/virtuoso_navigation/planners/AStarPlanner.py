# Imports
from collections import defaultdict
import numpy as np
from typing import Optional, Callable
from Planner import Planner
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import heapq

import math

MAX_INT32 = np.iinfo(np.int32).max


class AStarPlanner(Planner):
    def __init__(self, inflation_layer):
        super().__init__(inflation_layer)

    def create_path(self, goal: Pose) -> Path:
        start_pos: tuple[int, int] = self.to_map_pos(
            self.robot_pose.position.x, self.robot_pose.position.y
        )
        end_pos: tuple[int, int] = self.to_map_pos(goal.position.x, goal.position.y)
        grid = np.reshape(
            np.array(self.map.data), (self.map.info.height, self.map.info.width)
        )
        grid_path = self.a_star(grid, start_pos, end_pos)
        if not grid_path:
            raise Exception("No a_star path exists")
        ros_path = Path()
        pose_set = set()
        for node in grid_path:
            node = self.to_real_pos(node[0], node[1])
            p = PoseStamped()
            p.pose.position.x = node[0]
            p.pose.position.y = node[1]
            pose_set.add(p)
        ros_path.poses = pose_set
        return ros_path

    # Heutistic functions for a_star
    @staticmethod
    def euclidian_dist(
        start_point: tuple[int, int], end_point: tuple[int, int]
    ) -> float:
        """
        Gives the normal 2d Euclidian distance between 2 points
        """

        a = end_point[0] - start_point[0]
        b = end_point[1] - start_point[1]

        return np.sqrt(a * a + b * b)

    # convert x, y in meters to map coords
    def to_map_pos(self, pos_x: Optional[float] = 0, pos_y: Optional[float] = 0):
        grid_x, grid_y = 0, 0
        if self.map.info.resolution is float:
            grid_x = math.floor(pos_x / self.map.info.resolution)
            grid_y = math.floor(pos_y / self.map.info.resolution)

        grid_x = self.map.info.width // 2 + grid_x
        grid_y = self.map.info.height // 2 - grid_y

        if (grid_x > self.map.info.width or grid_x < 0) or (
            grid_y > self.map.info.height or grid_y < 0
        ):
            raise Exception("invalid grid_x or grid_y")

        return (grid_x, grid_y)

    # convert map coords to x, y in meters
    def to_real_pos(self, grid_x, grid_y):
        grid_x -= self.map.info.width // 2
        grid_y = self.map.info.height // 2 - grid_y
        grid_x *= self.map.info.resolution
        grid_y *= self.map.info.resolution
        return (grid_x, grid_y)

    def a_star(
        self,
        map_array: np.ndarray,
        start_pos: tuple[int, int],
        end_pos: tuple[int, int],
        h_func: Callable = euclidian_dist,
    ) -> list[tuple[int, int]] | None:
        def reconstruct_path(came_from, current):
            total_path = [current]
            while current in came_from:
                current = came_from[current]
                total_path.append(current)
            return total_path[::-1]

        def is_valid_neighbor(map_array: np.ndarray, neighbor: tuple[int, int]):
            for i in range(len(neighbor)):
                if neighbor[i] < 0 or neighbor[i] >= map_array.shape[i]:
                    return False
            if map_array[neighbor] > 0:
                return False
            return True

        dirs = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
        came_from = dict()
        # g_score[pos] is the cost of the cheapest path from start_pos to currently known pos
        g_score = defaultdict(lambda: float("inf"))
        g_score[start_pos] = 0
        # f_score[pos] = g_score[pos] + h_func(pos)
        f_score = defaultdict(lambda: float("inf"))
        f_score[start_pos] = h_func(start_pos)

        # initially, only start node is known. Implemented via min heap and set (for O(log n) pop and O(n) heap)
        open_heap = [(f_score[start_pos], start_pos)]
        open_set = {start_pos}

        while open_heap:
            _, pos = heapq.heappop(open_heap)
            open_set.remove(pos)
            if pos == end_pos:  # goal reached
                return reconstruct_path(came_from, pos)
            for i, j in dirs:
                neighbor = (pos[0] + i, pos[1] + j)
                if not is_valid_neighbor(map_array, neighbor):
                    continue
                grid_dist = (
                    1 if (i == 0 or j == 0) else np.sqrt(2)
                )  # if dir is a side, then distance 1, if corner than distance sqrt 2
                temp_gscore = g_score[pos] + grid_dist
                if temp_gscore < g_score[neighbor]:
                    # this path to neighbor is better than any previous one
                    came_from[neighbor] = pos
                    g_score[neighbor] = temp_gscore
                    f_score[neighbor] = temp_gscore + h_func(neighbor)
                    if neighbor not in open_set:
                        heapq.heappush(open_heap, (f_score[neighbor], neighbor))
                        open_set.add(neighbor)
        # open set empty but goal never reached
        return None
