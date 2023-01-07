import numpy as np

class PixelMatcher:

    # find the (left midpoint, right midpoint)
    # midpoints are (y, x)
    def midpoints(left_img:np.ndarray, right_img:np.ndarray):

        midpoints_indexes = [
            np.where(left_img == 255),
            np.where(right_img == 255)
        ]

        midpoint_counters = list(
            {'x-sum': 0, 'x-count': indexes[0].size, 'y-sum': 0, 'y-count': indexes[1].size} 
            for indexes in midpoints_indexes
        )

        for i in range(len(midpoints_indexes)):
            for j in range(midpoints_indexes[i][0].size):
                midpoint_counters[i]['x-sum'] += midpoints_indexes[i][1][j]
                midpoint_counters[i]['y-sum'] += midpoints_indexes[i][0][j]

        midpoints = list(
            (counter['y-sum'] // counter['y-count'], counter['x-sum'] // counter['x-count'])
            for counter in midpoint_counters
        )

        return midpoints[0], midpoints[1]