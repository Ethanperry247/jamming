import numpy as np

# Saving and loading to CSV files in common location.
def load_gradient(filename):
        return np.loadtxt(f'./maps/{filename}', delimiter=',')

def save_gradient(np_arr, filename):
        name = f'./maps/{filename}'
        # file = open(name, 'w+')
        # file.close()
        np.savetxt(name, np_arr, delimiter=',')

def create_random_gradient(rows, cols):
        return np.random.rand(rows, cols)

# Repeat a predefined np array across a particular number of rows and cols.
# Example:
# [0, 1, 2, 3] across 3, 2:
# [[0 1 2 3 0 1 2 3]
#  [0 1 2 3 0 1 2 3]
#  [0 1 2 3 0 1 2 3]]
def uniform_gradient(np_arr, rows, cols):
        return np.tile(np_arr, (rows, cols))

save_gradient(create_random_gradient(100, 100), 'example.csv')