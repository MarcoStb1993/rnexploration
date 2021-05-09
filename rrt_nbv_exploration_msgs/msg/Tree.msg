# Describes a tree constructed with RRT
std_msgs/Header header
string ns

int32 root   			# Position of the root node in array nodes
Node[] nodes    		# Array of all nodes in the tree
int32 node_counter   	# Number of nodes in the nodes array
int32 nearest_node		# Node the robot currently is nearest to

Node[] frontiers   		# Array of all stored frontiers
int32 frontier_counter  # Number of stored frontiers
