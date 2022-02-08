# A node in the graph constructed by RRG

int32 index								# Index of the node in the RRG node list
geometry_msgs/Point position			# Position of the node in the map
float64 gain    						# Calculated information gain metrics at this position for exploration (-1 if not calculated)
int32 best_yaw							# Yaw in degrees [0,360) with best gain 
float64 reward_function					# Reward function that combines distance to robot, heading, traversability, gain and radius
float64 radius							# Radius of the cleared area around the node
float64 squared_radius					# Squared Radius of the cleared area around the node
bool retry_inflation					# If the inflation can be retried because it previously failed due to unknown map tiles
uint8 status    						# Nodes current status regarding the exploration

uint8 INITIAL = 0       				# Node is waiting to be explored
uint8 ACTIVE = 1        				# Node is currently targeted as navigation goal
uint8 VISITED = 2 						# Node was visited but not completely explored
uint8 ACTIVE_VISITED = 3 				# Node is currently active and was visited but not completely explored before
uint8 EXPLORED = 4      				# Node was already explored
uint8 FAILED = 5        				# Node's exploration failed
uint8 ABORTED = 6        				# Node's exploration aborted

int32[] edges							# Position of the node's edges in the edges array of the graph message
int32 edge_counter						# Number of edges the node is connected to (also number of neighbors)

int32[] path_to_robot					# Position of the nodes that connect this node (end) to the node nearest to the robot (begin)
float64 distance_to_robot				# Distance along the graph's edges from this node to the node nearest to the robot
int32 heading_in						# Yaw angle in degrees from which the robot will go into this node from the shortest path
int32 heading_change_to_robot			# Accumulated change in heading along the graph's edges from this node to the node nearest to the robot in degrees(excluding change to best yaw)
int32 heading_change_to_robot_best_view # Accumulated change in heading along the graph's edges from this node to the node nearest to the robot in degrees
int32 traversability_cost				# Traversability cost for traversing this node
int32 traversability_weight				# Weight of traversing this node equal to the number of tiles in its radius
int32 traversability_cost_to_robot		# Accumulated traversability cost of nodes and edges from this node to the node nearest to the robot
int32 traversability_weight_to_robot	# Accumulated weight of traversing nodes and edges from this node to the node nearest to the robot equal to the number of tiles in its radius
float64 radii_to_robot					# Accumulated node radii from this node to the node nearest to the robot