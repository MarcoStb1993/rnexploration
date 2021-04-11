# Stores all currently active frontiers and the frontier blocklist
std_msgs/Header header
string ns

Frontier[] frontiers   				# Array of all frontiers
geometry_msgs/Point[] blocklist    	# Array of all frontiers
int32 frontier_counter				# Number of frontiers