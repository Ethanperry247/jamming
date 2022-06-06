import math

import numpy as np


class Map:
	def __init__(self, filepath) -> None:
		data = open(filepath)
		self.map = np.loadtxt(data, delimiter=",")
		print("Map:")
		print(self.map)

		# Map must be at least a 1x1 in size.
		if len(self.map) < 1 or len(self.map[0]) < 1:
			raise

	# Given a agent pose and in-memory map, calculate RSSI at current position.
	def rssi_from_current_pose(self, pose):
		x = pose['x']
		y = pose['y']
		print("RSSI At:", x, y)

		# Determine which discrete map points the robot is within
		x_l = math.floor(x)
		x_u = math.ceil(x)
		y_l = math.floor(y)
		y_u = math.ceil(y)

		# Find the x "middle point" on the lower y-side of the space
		x_mid_l = (self.map[x_u][y_l] - self.map[x_l][y_l]) * (x - math.floor(x)) + self.map[x_l][y_l]
		# Find the x "middle point" on the upper y-side
		x_mid_u = (self.map[x_u][y_u] - self.map[x_l][y_u]) * (x - math.floor(x)) + self.map[x_l][y_u]
		# Find "y averaged" point between the two x middle points
		retVal = (x_mid_u - x_mid_l) * (y - math.floor(y)) + x_mid_l
		print("Return: ", retVal)

		return retVal

	# Scan the map for the position of maximum intensity.
	def get_maximum_intensity_position(self):
		maxRSSI = self.map[0][0]
		pos = {
			'x': 0,
			'y': 0
		}
		for row_idx, row in enumerate(self.map):
			for col_idx, col in enumerate(row):
				if col > maxRSSI:
					pos = {
						'x': row_idx,
						'y': col_idx
					}
					maxRSSI = col
		return pos
