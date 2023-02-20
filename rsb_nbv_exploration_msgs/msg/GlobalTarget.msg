# A global target to store unexplored nodes outside of the local graph
int32 index							# Index of the target in the global graph target list
geometry_msgs/Point viewpoint		# Position from which the target can be seen
int32[] connections					# List of indices of the connections in the global graph between this target and other targets and the local graph
int32 connections_counter			# Number of connections between this target and others target and the local graph
float64 merged_distance				# Current maximum distance to targets merged into this target, 0 if none merged
bool inactive						# If this target was removed and is waiting to be replaced