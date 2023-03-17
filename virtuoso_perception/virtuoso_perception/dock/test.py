import numpy as np

arr = np.array([[0,0], [1,0], [2,0], [3,0], [3,1], [3,2], [2,2], [1,2], [0,2], [0,1]])

print(arr.shape)
# maxs = np.max(arr, axis=0)
# print(maxs)
# mins = np.min(arr, axis=1)
# print(mins)
max = np.min(arr[:,1])
print(max)

image = np.array([
    [[1,1,1], [2,2,2], [3,3,3]],
    [[4,4,4], [5,5,5], [6,6,6]],
    [[7,7,7], [8,8,8], [9,9,9]]
])

left = 1
right = 2
top = 1

above = image[top-1:top,left:right+1]
print(above)

print(np.reshape(above, (above.shape[0] * above.shape[1], 3)))
print(np.append(np.reshape(above, (-1, 3)), np.reshape(above, (-1, 3)), axis=0))

print(np.append(np.reshape(above, (-1, 3)), np.reshape(np.ndarray((0,0,3), dtype=int), (-1,3)), axis=0))