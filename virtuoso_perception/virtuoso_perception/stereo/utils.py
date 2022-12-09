import numpy as np

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