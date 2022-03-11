# Describes a global graph constructed with RRG
std_msgs/Header header
string ns

GlobalFrontier[] frontiers		  				# Array of all frontiers in this global graph with the first entry being the origin
GlobalPath[] paths								# Array of all paths connecting frontiers with each other and the local graph in this global graph
int32 frontiers_counter				   			# Number of frontiers in the frontiers array
int32 paths_counter   							# Number of paths in the paths array