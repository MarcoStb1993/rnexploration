# Describes a tree constructed with RRT
std_msgs/Header header
string ns

int32 root   # Position of the root node in array nodes
Node[] nodes    # Array of all nodes in the tree
int32 node_counter   #number of nodes in the nodes array
