# An edge in the graph constructed by RRG
int32 index						# Index of the edge in the RRG edge list
int32 first_node				# Index of the first node the edge connects (must always be of smaller index than second)
int32 second_node				# Index of the second node the edge connects (must always be of greater index than first)
float64 length    				# Length of the edge in m
int32 yaw						# Yaw of this edge from first to second node in degrees [0,360)
int32 traversability_cost		# Traversability cost for traversing this edge
int32 traversability_weight		# Weight of traversing this edge equal to the number of tiles in its path box