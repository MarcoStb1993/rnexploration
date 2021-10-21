# Metadata for a gain cluster of points for SRP
int32 size						# Number of points in the gain cluster
int32 index						# Index of the gain cluster for the particular node 
int32 node_index				# Index of the particular node
int32 frontier					# Number of the frontier cluster this gain cluster is a part of
int32 previous_frontier			# Number of the frontier cluster this gain cluster was a part of previously
SphericalPoint center			# Gain cluster center in spherical coordinates
SphericalPoint min_extent		# Minimum extent of the gain cluster in spherical coordinates
SphericalPoint max_extent		# Maximum extent of the gain cluster in spherical coordinates
geometry_msgs/Point position	# Position of the gain cluster in the Cartesian map