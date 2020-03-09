# A node in the RRT constructed tree
geometry_msgs/Point position
float64 gain    # Calculated information gain at this position for exploration
int32 best_yaw	# Yaw in degrees with best gain
uint8 status    # Nodes current status regarding the exploration

uint8 WAITING = 0       # Node is waiting to be explored
uint8 ACTIVE = 1        # Node is currently targeted as navigation goal
uint8 RECALCULATING = 2 # Gain will be recalculated
uint8 EXPLORED = 3      # Node was already explored
uint8 ABORTED = 4       # Node's exploration was aborted
uint8 FAILED = 5        # Node's exploration failed


int32[] children     # Position of the node's children in the nodes array of the RRT msg
int32 children_counter  # Number of children nodes in the children array
