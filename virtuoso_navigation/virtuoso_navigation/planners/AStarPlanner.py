# Imports 
import numpy as np
from Planner import Planner
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Path

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
        start_pos: tuple[int, int] = self.position_to_node(self.robot_pose.position.x, self.robot_pose.position.y)
        end_pos: tuple[int, int] = self.position_to_node(goal.position.x, goal.position.y)

        map_array: np.array = self.occupancy_grid_to_nparray(self.map)
        
        grid_path: list[tuple[int, int]] = self.a_star(map_array, start_pos, end_pos)
        
        return self.construct_path(grid_path)     
        
    # Heutistic functions for a_star
    @staticmethod
    def euclidian_dist(start_point: tuple[int, int], end_point: tuple[int, int]) -> float:
        """
        Gives the normal 2d Euclidian distance between 2 points
        """

        a = end_point[0] - start_point[0]
        b = end_point[1] - start_point[1]

        return np.sqrt(a*a + b*b) 

    def occupancy_grid_to_nparray(occupancy_grid: OccupancyGrid):
        """
        Convert a ROS2 OccupancyGrid message to a numpy array.

        Args:
        occupancy_grid (nav_msgs.msg.OccupancyGrid): The OccupancyGrid message.

        Returns:
        numpy.ndarray: A 2D numpy array representing the occupancy grid.
        """
        # Get the width and height from the OccupancyGrid
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height

        # Convert the data to a numpy array
        data = np.array(occupancy_grid.data)

        # Reshape the numpy array to 2D based on width and height of the grid
        return np.reshape(data, (height, width))
 
    # Util functions for other nodes to interface with
    def position_to_node(occupancy_grid: OccupancyGrid, pos_x: float, pos_y: float):
        grid_x = math.floor(pos_x / occupancy_grid.resolution)
        grid_y = math.floor(pos_y / occupancy_grid.resolution)

        grid_x = occupancy_grid.width // 2 + grid_x
        grid_y = occupancy_grid.height // 2 - grid_y

        if (grid_x > occupancy_grid.width or grid_x < 0) or (grid_y > occupancy_grid.height or grid_y < 0):
            return None
        
        return (grid_x, grid_y)

    def a_star(self, map_array: np.array, start_pos: tuple[int, int],
               end_pos: tuple[int, int],
               h_func: callable = euclidian_dist) -> list[tuple[int, int]]:

        open_close_set: np.array = np.full(map_array.shape, np.NaN)
        open_close_set[start_pos] = SquareTypes.OPEN_SET

        came_from: np.array = np.full(map_array.shape, np.NaN, dtype=np.int32)

        g_score = np.full(map_array.shape, MAX_INT32, dtype=np.int32)
        g_score[start_pos] = 0

        f_score = np.full(map_array.shape, np.Inf)
        f_score[start_pos] = h_func(start_pos, end_pos)

        while np.any(open_close_set == SquareTypes.OPEN_SET):
            f_mask = f_score.copy();
            f_mask[open_close_set != 1] = np.Inf
            
            current = np.unravel_index(np.argmin(f_mask), map_array)
            
            if current == end_pos:
                return self.reconstruct_gridpath(came_from, start_pos, end_pos, map_array.shape)
            
            open_close_set[current] = 2
            
            for i in range(-1, 2):
                for j in range(-1, 2):
                    neighbor = (current[0] + i, current[1] + j)
                    if (i == 0 and j == 0) or not self.is_valid_neighbor(map_array, neighbor):
                        continue
                    
                    grid_dist = 1 if (i == 0 or j == 0) else np.sqrt(2)
                    
                    temp_g = g_score[current] + grid_dist
                    
                    
                    if temp_g < g_score[neighbor]:
                        came_from[neighbor] = np.ravel_multi_index(current, map_array.shape)
                        
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
    

    def reconstruct_gridpath(came_from: np.array, start_pos: tuple[int, int], 
                         end_pos: tuple[int, int], grid_shape: tuple[int, int]) -> list[tuple[int, int]]:
        
        current = end_pos
        path = [current]

        while path[-1] != start_pos:
            current = np.unravel_index(came_from[current], grid_shape)
            path.append(current)

        return path

    def construct_path(grid_path: list[tuple[int, int]]) -> Path:
        pass
