import numpy as np

t45 = np.array([
    [1, 0, 0, -0.015],
    [0, 1, 0, 0.045],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

print(t45[0:3,3].reshape(-1,1))