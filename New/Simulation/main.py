import math
import matplotlib.pyplot as plt
import networkx as nx
from Utils.agent_utils import Agent
from Utils.map_utils import Map


# Robot follows RSSI threshold to the right at all times. 
def main(threshold, epsilon, policy):
	agent = Agent(20, policy['step_size'], policy['initial_pose'])
	map = Map('./Maps/map_1.csv')

	step_count = 0
	while not agent.check_polygon_complete():
		try:
			# Calculates the RSSI measured by the agent in its current position.
			current_rssi = agent.measure_rssi(map)
		except Exception:
			break

		# Identifies the angle of maximum intensity, or the angle from the robot to the jammer source.
		maximum_intensity = agent.get_maximum_intensity_angle(map)

		# Check if we are at the desired threshold
		if threshold - epsilon < current_rssi < threshold + epsilon:
			# Add this point to the polygon
			agent.append_polygon()
			# Move adjacent to jammer
			agent.orient(maximum_intensity - (math.pi / 2))
		elif current_rssi < threshold:
			# Not at threshold, move towards jammer
			agent.orient(maximum_intensity)
		else:
			# Over desired threshold, move backwards
			agent.orient(-maximum_intensity)
		agent.take_step()
		step_count += 1

	print("Done!")

	# Draw the path that the robot took
	path_graph = nx.Graph()
	final_path = agent.path
	color_map = []
	for idx, node in enumerate(final_path):
		path_graph.add_node(idx, pos=(node['x'], node['y']))
		if idx == 0:
			color_map.append('green')
		else:
			path_graph.add_edge(idx - 1, idx)
			color_map.append('gray')
	pos = nx.get_node_attributes(path_graph, 'pos')
	nx.draw(path_graph, pos, node_color=color_map, with_labels=False, node_size=10)

	# Draw the polygon that circles the jammer
	polygon_graph = nx.Graph()
	final_polygon = agent.get_polygon()
	color_map = []
	for idx, node in enumerate(final_polygon):
		polygon_graph.add_node(idx, pos=(node['x'], node['y']))
		if idx != 0:
			polygon_graph.add_edge(idx - 1, idx)
		color_map.append('blue')
	max_intensity_pos = map.get_maximum_intensity_position()
	polygon_graph.add_node(len(final_polygon), pos=(max_intensity_pos['x'], max_intensity_pos['y']))
	color_map.append('red')
	pos = nx.get_node_attributes(polygon_graph, 'pos')
	nx.draw(polygon_graph, pos, node_color=color_map, with_labels=False, node_size=15)

	# Show-off our awesome solution!
	plt.xlim([-1, 8])
	plt.ylim([-1, 8])
	plt.show()


# Step size and angle adjustment determine the turning speed and step size for one iteration of the robots movement.
main(5.5, 1, {
	'angle_adjustment': math.pi / 16,
	'step_size': 0.1,
	'initial_pose': {
		'x': 0,
		'y': 0,
		'theta': 0,
		'forward': 0
	}
})
