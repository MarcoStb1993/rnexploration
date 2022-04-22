# A frontier to store unexplored nodes outside of the local graph
int32 index							# Index of the frontier in the global graph frontier list
geometry_msgs/Point viewpoint		# Position from which the frontier can be seen
int32[] paths						# List of indices of the paths in the global graph connecting this frontier to others and the local graph
int32 paths_counter					# Number of paths connecting this frontier to others and the local graph
float64 merged_distance				# Current maximum distance to frontiers merged into this frontier, 0 if none merged
bool inactive						# If this frontier was removed and is waiting to be replaced