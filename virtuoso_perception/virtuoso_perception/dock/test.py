import numpy as np

arr = np.array([[0,0], [1,0], [2,0], [3,0], [3,1], [3,2], [2,2], [1,2], [0,2], [0,1]])

print(arr.shape)
# maxs = np.max(arr, axis=0)
# print(maxs)
# mins = np.min(arr, axis=1)
# print(mins)
max = np.min(arr[:,1])
print(max)