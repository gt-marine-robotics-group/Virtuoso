from nav_msgs.msg import OccupancyGrid

class InflationLayer:

    def __init__(self, size: float):
        self._size = size

        self.map: OccupancyGrid = None
    
    def in_inflation_layer(self, x, y):

        # x and y are in map frame

        if self.map.info.resolution < self._size:
            for i in range(1, int(self._size / self.map.info.resolution) + 1):
                s = self.map.info.resolution * i
                points = [
                    (x + s, y),
                    (x, y + s),
                    (x - s, y),
                    (x, y - s),
                    (x + s * 0.707, y + s * 0.707),
                    (x - s * 0.707, y + s * 0.707),
                    (x - s * 0.707, y - s * 0.707),
                    (x + s * 0.707, y - s * 0.707)
                ]

                for point in points:
                    if self._is_occupied(*point):
                        return True
                
            return False

        
        points = [
            (x + self._size, y),
            (x, y + self._size),
            (x - self._size, y),
            (x, y - self._size),
            (x + self._size * 0.707, y + self._size * 0.707),
            (x - self._size * 0.707, y + self._size * 0.707),
            (x - self._size * 0.707, y - self._size * 0.707),
            (x + self._size * 0.707, y - self._size * 0.707)
        ]

        for point in points:
            if self._is_occupied(*point):
                return True
        
        return False

    def _is_occupied(self, x: float, y: float):
        # mostly copied from RRT.py in virtuoso_navigation
        # ideally should be some utility function

        # Note that x and y are coming in the map frame but we should not index the costmap
        # with that x and y. It must be transformed to the costmap indices. 

        self.width_m = self.map.info.width * self.map.info.resolution
        self.height_m = self.map.info.height * self.map.info.resolution

        x_costmap = (x + (self.width_m / 2)) / self.map.info.resolution
        y_costmap = -1 * ((self.height_m / 2) - y) / self.map.info.resolution

        x_index = int(x_costmap)
        y_index = int(y_costmap) - 1

        index = (y_index * self.map.info.width) + x_index

        if index >= len(self.map.data) or index < 0:
            return False

        if self.map.data[index] > 0:
            return True
        
        return False
