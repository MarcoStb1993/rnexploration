# A node in the RRT constructed tree
int32 index						# Index of the node in the RRT node list
geometry_msgs/Point position	# Position of the node in the map
float64 gain    				# Calculated information gain metrics at this position for exploration (-1 if not calculated)
int32 best_yaw					# Yaw in degrees with best gain
uint8 status    				# Nodes current status regarding the exploration

uint8 INITIAL = 0       		# Node is waiting to be explored
uint8 ACTIVE = 1        		# Node is currently targeted as navigation goal
uint8 VISITED = 2 				# Node was visited but not completely explored
uint8 ACTIVE_VISITED = 3 		# Node is currently active and was visited but not completely explored before
uint8 EXPLORED = 4      		# Node was already explored
uint8 FAILED = 5        		# Node's exploration failed
uint8 ABORTED = 6        		# Node's exploration aborted

int32 parent					# Position of the node's parent in the nodes array of the RRT msg (-1 for root)
int32[] children     			# Position of the node's children in the nodes array of the RRT msg
int32 children_counter  		# Number of children nodes in the children array

int32[] pathToRobot				# Position of the nodes that connect this node (end) to the node nearest to the robot (begin)
float64 distanceToRobot			# Distance along the tree's edges from this node to the node nearest to the robot
float64 distanceToParent		# Distance in m to this node's parent node			