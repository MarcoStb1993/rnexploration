# A frontier found in the map
uint32 size						# Amount of grid cells in the frontier
geometry_msgs/Point initial		# Nearest position to the robot of the frontier in the map
geometry_msgs/Point centroid	# Central position of the frontier in the map
geometry_msgs/Point[] points	# List of all grid cells in the frontier
int32 index						# Index of the frontier in the list of frontiers
float64 gain    				# Calculated information gain metrics at this position for exploration (-1 if not calculated)
int32 best_yaw					# Yaw in degrees with best gain
uint8 status    				# Frontiers current status regarding the exploration
float64 distanceToRobot			# Distance from the frontier's centroid to the robot

uint8 INITIAL = 0       		# Frontier is waiting to be explored
uint8 ACTIVE = 1        		# Frontier is currently targeted as navigation goal
uint8 VISITED = 2 				# Frontier was visited but not completely explored
uint8 ACTIVE_VISITED = 3 		# Frontier is currently active and was visited but not completely explored before
uint8 EXPLORED = 4      		# Frontier was already explored
uint8 FAILED = 5        		# Frontier exploration failed
uint8 ABORTED = 6        		# Frontier exploration aborted