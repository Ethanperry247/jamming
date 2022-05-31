import numpy as np
import math

class Map():
    def __init__(self, filepath) -> None:
        data = open(filepath)
        self.map = np.loadtxt(data, delimiter=",")

        # Map must be at least a 1x1 in size.
        if (len(self.map) < 1 or len(self.map[0]) < 1):
            raise

    # Reads map into memory from given filepath.
    def read_map(filepath):
        pass

    # Given a agent pose and in-memory map, calculate RSSI at current position.
    def rssi_from_current_pose(self, pose):
        x = pose['x']
        y = pose['y']
        
        upper_x = math.ceil(x)
        upper_y = math.ceil(y)
        lower_x = math.floor(x)
        lower_y = math.floor(y)

        x_diff_lower = self.map[lower_x][lower_y] - self.map[upper_x][lower_y]
        x_diff_lower_adjusted = x_diff_lower * (x % 1)

        x_diff_upper = self.map[lower_x][upper_y] - self.map[upper_x][upper_y]
        x_diff_upper_adjusted = x_diff_upper * (x % 1)

        x_diff = x_diff_upper_adjusted - x_diff_lower_adjusted
        return x_diff * (y % 1)


    # Scan the map for the position of maximum intensity.
    def get_maximum_intensity_position(self):
        max = self.map[0][0]
        pos = {
            'x': 0,
            'y': 0
        }
        for row_idx, row in enumerate(self.map):
            for col_idx, col in enumerate(row):
                if col > max:
                    pos = {
                        'x': row_idx,
                        'y': col_idx
                    }
        
        return pos
