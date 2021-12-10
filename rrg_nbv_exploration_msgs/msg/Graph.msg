# Describes a graph constructed with RRG
std_msgs/Header header
string ns

Node[] nodes			    		# Array of all nodes in the graph
int32 node_counter				   	# Number of nodes in the nodes array
int32 nearest_node					# Node the robot currently is nearest to
Edge[] edges						# Array of all edges in the graph
int32 edge_counter   				# Number of edges in the edges array
float64 longest_distance_to_robot	# Longest path from robot to a node along the edges in m
