# Describes a graph constructed with RRG
std_msgs/Header header
string ns

Node[] nodes			    					# Array of all nodes in the graph
int32 node_counter				   				# Number of nodes in the nodes array
int32 nearest_node								# Node the robot currently is nearest to
Edge[] edges									# Array of all edges in the graph
int32 edge_counter   							# Number of edges in the edges array
float64 longest_distance_to_robot				# Longest path from robot to any node in the graph along the edges in m
float64 highest_node_gain						# Highest gain of any node in the graph
float64 largest_heading_change_to_robot_best_view	# Largest accumulated heading change of any node's best view in the graph to the robot in deg
float64 highest_traversability_cost_to_robot	# Highest accumulated, traversability cost per tile of any node in the graph to the robot
float64 largest_node_radius						# Largest radius of any node in the graph in m