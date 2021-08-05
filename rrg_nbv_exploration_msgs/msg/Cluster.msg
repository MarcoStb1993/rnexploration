# Metadata for a cluster of points for SRP
int32 size						# Number of points in the cluster
int32 index						# Index of the cluster for the particular node 
SphericalPoint center			# Cluster center in spherical coordinates
SphericalPoint min_extent		# Minimum extent of the cluster in spherical coordinates
SphericalPoint max_extent		# Maximum extent of the cluster in spherical coordinates
geometry_msgs/Point position	# Position of the cluster in the Cartesian map