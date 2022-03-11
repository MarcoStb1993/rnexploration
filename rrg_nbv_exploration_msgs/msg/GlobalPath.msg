# A path connecting frontiers to each other or the local graph
int32 index						# Index of the path in the global graph path list
int32 frontier					# Index of the frontier this path is connected to (always higher index than connection)
int32 connection				# Index of a second frontier this path connects the first with, the origin (0) or the local graph (-1)

int32 ORIGIN = 0       			# Path is connected to the origin of the global graph
int32 LOCAL_GRAPH = -1        	# Path is connected to the local graph

int32 connecting_node			# Index of a node (local graph) connected to this path or -1 for the origin
float64 length    				# Length of the path in m
geometry_msgs/Point[] waypoints	# List of waypoints from frontier to local graph/second frontier
bool inactive					# If frontier connected by this path was removed and path is waiting to be replaced