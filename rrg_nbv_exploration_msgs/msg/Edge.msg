# An edge in the graph constructed by RRG
int32 index						# Index of the edge in the RRG edge list
int32 first_node				# Index of the first node the edge connects (must always be of smaller index than second)
int32 second_node				# Index of the second node the edge connects (must always be of greater index than first)
float64 length    				# Length of the edge in m
float64 yaw						# Yaw of this edge from first to second node
float64 traversability_cost		# Normalized cost for traversing this edge