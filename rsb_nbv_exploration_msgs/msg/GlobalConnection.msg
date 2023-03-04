# A connection between global targets or a global target and the local graph
int32 index						# Index of the connection in the global graph connection list
int32 first_target				# Index of the first target of this conection (always higher index than connection)
int32 second_target				# Index of the second target of this conection (0=origin, -1=local graph)

int32 ORIGIN = 0       			# Connection to the origin of the global graph
int32 LOCAL_GRAPH = -1        	# Connection to the local graph

int32 connecting_node			# Index of a node in the local graph that the connection leads to if the second target is the local graph
float64 length    				# Length of the path in m
geometry_msgs/Point[] waypoints	# List of waypoints from first target to second target
bool inactive					# If a target from this connection was removed and the connection is waiting to be replaced