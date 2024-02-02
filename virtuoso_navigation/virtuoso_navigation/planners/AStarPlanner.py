# Imports
from collections import defaultdict
import numpy as np
from typing import Optional, Callable
from Planner import Planner
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Path
import heapq

import math
from enum import Enum

MAX_INT32 = np.iinfo(np.int32).max


class SquareTypes(Enum):
    """
    Holds the values that will be used while a_star calculates the optimal path
    """

    EMPTY_SQUARE = 0
    OBSTACLE = -1
    OPEN_SET = 1
    CLOSED_SET = 2


class AStarPlanner(Planner):
    def __init__(self, inflation_layer):
        super().__init__(inflation_layer)

    def create_path(self, goal: Pose) -> Path:
        start_pos: tuple[int, int] = self.map_pos(
            self.robot_pose.position.x, self.robot_pose.position.y
        )
        end_pos: tuple[int, int] = self.map_pos(goal.position.x, goal.position.y)
        width = self.map.info.width
        height = self.map.info.height
        grid = np.reshape(np.array(self.map.data), (height, width))
        grid_path: list[tuple[int, int]] = self.a_star(grid, start_pos, end_pos)

        return self.construct_path(grid_path)

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

    # Util functions for other nodes to interface with
    def map_pos(self, pos_x: Optional[float] = 0, pos_y: Optional[float] = 0):
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

    def a_star(
        self,
        map_array: np.ndarray,
        start_pos: tuple[int, int],
        end_pos: tuple[int, int],
        h_func: Callable = euclidian_dist,
    ) -> list[tuple[int, int]]:
        came_frm = dict()
        g_scor = defaultdict(lambda: float("inf"))
        g_scor[start_pos] = 0
        f_scor = defaultdict(lambda: float("inf"))
        f_scor[start_pos] = h_func(start_pos)
        open_heap = [(f_scor, start_pos)]

        while open_heap:
            f, pos = heapq.heappop(open_heap)
            if pos == end_pos:
                return 0

        open_close_set: np.ndarray = np.full(map_array.shape, np.NaN)
        open_close_set[start_pos] = SquareTypes.OPEN_SET

        came_from: np.ndarray = np.full(map_array.shape, np.NaN, dtype=np.int32)

        g_score = np.full(map_array.shape, MAX_INT32, dtype=np.int32)
        g_score[start_pos] = 0

        f_score = np.full(map_array.shape, np.Inf)
        f_score[start_pos] = h_func(start_pos, end_pos)

        while np.any(open_close_set == SquareTypes.OPEN_SET):
            f_mask = f_score.copy()
            f_mask[open_close_set != 1] = np.Inf

            current = np.unravel_index(np.argmin(f_mask), map_array)

            if current == end_pos:
                return self.reconstruct_gridpath(
                    came_from, start_pos, end_pos, map_array.shape
                )

            open_close_set[current] = 2

            for i in range(-1, 2):
                for j in range(-1, 2):
                    neighbor = (current[0] + i, current[1] + j)
                    if (i == 0 and j == 0) or not self.is_valid_neighbor(
                        map_array, neighbor
                    ):
                        continue

                    grid_dist = 1 if (i == 0 or j == 0) else np.sqrt(2)

                    temp_g = g_score[current] + grid_dist

                    if temp_g < g_score[neighbor]:
                        came_from[neighbor] = np.ravel_multi_index(
                            current, map_array.shape
                        )

                        g_score[neighbor] = temp_g
                        f_score[neighbor] = temp_g + h_func(neighbor, end_pos)

                        open_close_set[neighbor] = 1

        return None

    def is_valid_neighbor(map_array: np.array, neighbor: tuple[int, int]):
        for i in range(len(neighbor)):
            if neighbor[i] < 0 or neighbor[i] >= map_array.shape[i]:
                return False

        if map_array[neighbor] == SquareTypes.OBSTACLE:
            return False

        return True

    def reconstruct_gridpath(
        came_from: np.array,
        start_pos: tuple[int, int],
        end_pos: tuple[int, int],
        grid_shape: tuple[int, int],
    ) -> list[tuple[int, int]]:
        current = end_pos
        path = [current]

        while path[-1] != start_pos:
            current = np.unravel_index(came_from[current], grid_shape)
            path.append(current)

        return path

    def construct_path(grid_path: list[tuple[int, int]]) -> Path:
        pass
