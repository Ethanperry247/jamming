import math

import matplotlib.pyplot as plt
import networkx as nx

from Utils.agent_utils import Agent
from Utils.map_utils import Map


# Robot follows RSSI threshold to the right at all times. 
def main(threshold, epsilon, policy):
	agent = Agent(20, policy['step_size'], policy['initial_pose'])
	map = Map('./Maps/map_1.csv')

	# Add the robot's starting position to the polygon
	agent.append_polygon()

	while (not agent.check_polygon_complete()):
		try:
			# Calculates the RSSI measured by the agent in its current position.
			current_rssi = agent.measure_rssi(map)
		except Exception:
			break

		# Identifies the angle of maximum intensity, or the angle from the robot to the jammer source.
		maximum_intensity = agent.get_maximum_intensity_angle(map)

		# Current position of the agent.
		current_pose = agent.get_pose()

		difference = abs((maximum_intensity - current_pose['forward'])) % (math.pi / 2)
		adjusted_difference = difference / (math.pi / 2)

		# Change the rotation of the agent to the angle of maximum intensity.
		agent.rotate_absolute(maximum_intensity)

		if (current_rssi > threshold + epsilon):
			# Reorient robot and move away from threshold.
			agent.orient((policy['angle_adjustment']) * adjusted_difference)

		if (current_rssi < threshold - epsilon):
			# Move towards the threshold.
			agent.orient(-(policy['angle_adjustment'] * adjusted_difference))

		agent.take_step()
		agent.append_polygon()

	final_polygon = agent.get_polygon()
	print("Done!")
	print(final_polygon)

	G = nx.Graph()
	color_map = []
	for idx, node in enumerate(final_polygon):
		G.add_node(idx, pos=(node['x'], node['y']))
		if (idx == 0):
			G.add_edge(len(final_polygon) - 1, 0)
			color_map.append('green')
		else:
			G.add_edge(idx - 1, idx)
			color_map.append('blue')

	max_intensity_pos = map.get_maximum_intensity_position()
	G.add_node(len(final_polygon), pos=(max_intensity_pos['x'], max_intensity_pos['y']))
	color_map.append('red')

	pos = nx.get_node_attributes(G, 'pos')

	nx.draw(G, pos, node_color=color_map, with_labels=False)
	plt.show()


# Step size and angle adjustment determine the turning speed and step size for one iteration of the robots movement.
main(5.5, 1, {
	'angle_adjustment': math.pi / 16,
	'step_size': 0.1,
	'initial_pose': {
		'x': 1.5,
		'y': 1.5,
		'theta': 0,
		'forward': 0
	}
})
