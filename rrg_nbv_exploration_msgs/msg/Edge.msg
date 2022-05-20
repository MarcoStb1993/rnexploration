# An edge in the graph constructed by RRG
int32 index						# Index of the edge in the RRG edge list
int32 first_node				# Index of the first node the edge connects (must always be of smaller index than second)
int32 second_node				# Index of the second node the edge connects (must always be of greater index than first)
float64 length    				# Length of the edge in m (Distance from first to second node)
int32 yaw						# Yaw of this edge from first to second node in degrees [0,360)
float64 traversability_cost		# Traversability cost for traversing this edge
bool inactive    				# If the edge is outside of the active boundary and will be replaced with a new edge soon