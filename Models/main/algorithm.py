import math
import numpy as np
from bresenham import bresenham

# Radian difference between antenna measurements. Adjust based on physical parameters.
ANTENNA_ANGLE_DIFFERENCE = math.pi / 9
RSS_THRESHOLD_HIGH = 10
RSS_THRESHOLD_LOW = 5
IDEAL_RSS = (RSS_THRESHOLD_HIGH - RSS_THRESHOLD_LOW) / 2
uncertainty = 0

# Map describes the state of each cell (uncertain, discovered, free, occupied) / (0, 1, 2, 3).
# Map is filled with zeros upon initialization.
map = []

# Specify the desired dimensions of the exporation space.
x_dimension, y_dimension = 7

# Movements describe the (x, y) movement on the map.
movements = [(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1)]

# While the source is not normally known, it will be parameterized for this abstract algorithm.
source_location = (3, 3)

# Example field. Use gradient generation methods or create a different gradient if one is desired. 
field = [
        [6, 6, 6, 6, 6, 6, 6],
        [6, 6, 6, 7, 6, 6, 6],
        [6, 6, 7, 8, 7, 6, 6],
        [6, 7, 8, 9, 8, 7, 6],
        [6, 6, 7, 8, 7, 6, 6],
        [6, 6, 6, 7, 6, 6, 6],
        [6, 6, 6, 6, 6, 6, 6]
]

# Implemented based on hardware setup -- dependent on the method of measuring RSSI.
# For abstract purposes, find the angle between the source and the current location of the robot. 
# Will be specified in radians 0 to math.pi * 2.
def obtain_maximum_rss_angle(x, y):
        return math.atan2(source_location[1] - y, source_location[0] - x)
        

# Obtain an RSSI measurement at the current location. 
def measure_rssi(x, y):
        return field[x][y]

# Adjusts maximum rss angle by math.pi / 2 to find the tangent. 
def relative_movement_angle(x, y):
        return obtain_maximum_rss_angle(x, y) + (math.pi / 2)

# Return the direction to move next based on the tangent angle.
def relative_movement(angle):
        if ((angle > 0 and angle < math.pi / 8) or (angle < 2 * math.pi and angle > (15 * math.pi) / 8)):
                return movements[0]
        else:
                return movements[int((angle - math.pi / 8) / (2 * math.pi))]

"""
The robot grid-world is defined using x and y dimensions. The robot will use RSSI sample to navigate a gradient around a particular area. If obstacles are found in the robot's path, the robot will navigate around them until a certain RSSI threshold is reached. If the threshold is hit, the robot must turn around to go the other way around the obstacle, even if that means leaving the desired RSSI lower threshold. 

The algorithm is still underdeveloped at the current time. Obstacles and source estimation will be integrated next. 
"""
def begin():
        uncertainty = x_dimension * y_dimension
        map = np.zeros((x_dimension, y_dimension), dtype=int)

        step(0, 0, 0)

def step(time, x, y):
        if uncertainty == 0:
                return

        movement = relative_movement(relative_movement_angle(x, y))

        # Use bresenham's algorithm to point occupied zones.
        segment = bresenham(x, y, source_location[0], source_location[1])
        for point in segment:
                map[point[0], point[1]] = 2

                # Decrease the uncertainty with each occupied zone paint.
                uncertainty -= 1

        # TODO: Paint anything outside of the encircled RSSI region with free zones in the map layer.

        step(time + 1, x + movement[0], y + movement[1])