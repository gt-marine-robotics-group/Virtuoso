import numpy as np

y_points = np.arange(10, 50)
x_points = np.arange(100, 130)

contour = np.zeros((2 * y_points.shape[0] + 2 * x_points.shape[0] - 3, 2))

contour[:y_points.shape[0],0] = y_points
contour[:y_points.shape[0],1] = x_points[0]

contour[y_points.shape[0]:y_points.shape[0] + x_points.shape[0] - 1,0] = y_points[-1]
contour[y_points.shape[0]:y_points.shape[0] + x_points.shape[0] - 1,1] = np.arange(x_points[0] + 1, x_points[-1] + 1)

contour[y_points.shape[0] + x_points.shape[0] - 1:2 * y_points.shape[0] + x_points.shape[0] - 2,0] = np.arange(y_points[-1] - 1, y_points[0] - 1, -1)
contour[y_points.shape[0] + x_points.shape[0] - 1:2 * y_points.shape[0] + x_points.shape[0] - 2,1] = x_points[-1]

contour[2 * y_points.shape[0] + x_points.shape[0] - 2:,0] = y_points[0]
contour[2 * y_points.shape[0] + x_points.shape[0] - 2:,1] = np.arange(x_points[-1] - 1, x_points[0] - 1, -1)

print(contour)