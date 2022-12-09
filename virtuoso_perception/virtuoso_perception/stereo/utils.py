import numpy as np
import math

def unflatten_contours(flat_contours:list, contour_offsets:list):
    contours = np.empty((len(contour_offsets),), dtype=object)

    for i, offset in enumerate(contour_offsets):
        if i == len(contour_offsets) - 1:
            final_i = len(flat_contours)
        else:
            final_i = contour_offsets[i + 1]
        
        contour = np.empty(((final_i - offset) // 2, 1, 2), dtype='int64')
        flat_i = offset
        count = 0
        while flat_i < final_i:
            contour[count,0,0] = flat_contours[flat_i]
            contour[count,0,1] = flat_contours[flat_i + 1]
            count += 1
            flat_i += 2

        contours[i] = contour
    
    return contours

def _find_left_cam_x_angle(point:tuple, f:float, center:tuple):
    if point[1] > center[1]:
        return math.atan(f / (point[1] - center[1]))

    if point[1] < center[1]:
        return math.atan((center[1] - point[1]) / f) + (math.pi / 2)

    return math.pi / 2

def _find_right_cam_x_angle(self, point:tuple, f:float, center:tuple):
    if point[1] > center[1]:
        return math.atan((point[1] - center[1]) / f) + (math.pi / 2)
    
    if point[1] < center[1]:
        return math.atan(f / (center[1] - point[1]))
    
    return math.pi / 2

# fx1 and fx2 should be the same
def img_points_to_physical_xy(mid1:tuple, mid2:tuple, fx1:float, fx2:float, center:tuple,
    cam_separation=0.2):
    left_x_theta = _find_left_cam_x_angle(mid1, fx1, center)
    right_x_theta = _find_right_cam_x_angle(mid2, fx2, center)

    # self.get_logger().info(f'left theta: {left_x_theta * 180 / math.pi}')
    # self.get_logger().info(f'right theta: {right_x_theta * 180 / math.pi}')

    s_theta = math.pi - left_x_theta - right_x_theta

    left_hyp = math.sin(right_x_theta) * cam_separation / s_theta

    object_x = left_hyp * math.sin(left_x_theta)

    object_y = math.sqrt(left_hyp**2 - object_x**2)
    if mid1[1] > center[1]: object_y *= -1

    # self.get_logger().info(f'object x: {object_x}')
    # self.get_logger().info(f'object y: {object_y}')

    return object_x, object_y