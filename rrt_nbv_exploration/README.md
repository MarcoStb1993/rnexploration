# RRT NBV Exploration

This package contains an exploration algorithm that aims at enabling 3D mapping for ground robots. First, an introduction to the package is shown. Then the [RNE's implementation](#documentation) will be explained, followed by a list of its [nodes](#nodes) with the particular parameters, published and subscribed topics.

## Introduction

The RNE is based on an [RRT](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) which builds up by creating new nodes at randomly sampled points. The new nodes have to be connected to the tree's nearest neighbor. Therefore, a method checks if the robot would be able to traverse between the new node and its nearest neighbor in the tree.

Additionally, nodes can be sampled around the robot's current position which aids at exploring large areas.

To efficiently explore the environment, the node providing the most information is chosen as the next exploration goal. This is evaluated by calculating its gain and cost and combining them in a function to compare all nodes. The gain is computed by a form of raycasting in an [OctoMap ](http://wiki.ros.org/octomap) and the cost is based on the distance from the robot to the particular node measured only along the tree's edges.

Each node in the tree stores its position in the map, its parent and children, its gain as well as the yaw direction for which the gain was computed, the path along the tree's edges to the robot and its state. The state can be one of the states shown in the State Diagram below.

![Node states State Diagram](../images/NodeStateDiagram.png)

The NBV is determined by choosing the node with the best gain-cost-ratio (GCR). The gain calculation uses Sparse Ray Polling (SRP) in which sample points are determined by the utilized sensor's FoV, range and the step size defined by the user. This should be set regarding the OctoMap's voxel's edge length and the computing power of the system running the RNE. This set of sample points is calculated at the RNE's initialization, so that during its execution the sample points can be placed at the particular node to be evaluated by adding the node's position to each sample point. This and the execution in a node separate from the RRT construction aims to speed up the exploration. Below a set of sample points is employed to evaluate a node. Blue denotes unknown space, green free space and red occupied space. The particular ray is not evaluated further when an obstacle was detected. Also, the best yaw orientation for the robot at this node is calculated by adding up all sampling slices that could be observed if the sensor is pointed at this particular direction. The direction with the highest combined gain is set as the best yaw. The horizontal FoV of the sensor is required for this.

![Sparse Ray Sampling](../images/raysampling.png)

The RNE can be run in *finish* and *interrupt* mode. In the former each exploration goal must be reached or reaching it must fail before the next goal is chosen. The latter mode allows a current goal to be aborted when a better goal was found. Since RRT construction and gain calculation are decoupled, a new goal is already selected while some node's gains are still being computed. Therefore, the *interrupt* mode allows for changing the goal when a node with a better gain becomes available.

## Documentation

This package contains the RNE's core logic which is divided into the classes you can see in the image below. The Class Diagram is not complete but just shows the most important features of the RNE implementation.

![RNE Class Diagram](../images/ClassDiagram.png)

### TreeConstructor

The class TreeConstructor includes the RRT algorithm which builds the tree in sample space defined by the given OctoMap's dimensions. It manages and publishes the tree's current state and maintains an ordered list of all node's gains. This list is used to determine the node with the best gain which is proposed as the next goal to explore. When a goal was explored, it triggers the update of all node's in two times the sensor's range. The nodes to be updated are published in a topic and the updated nodes are subscribed to. The TreeConstructor also includes services to start and stop the exploration. It publishes the current goal node and the node with the best gain.

### TreeSearcher

The TreeSearcher is an interface to [nanoflann](https://github.com/jlblancoc/nanoflann#nanoflann) which is header only library for building k-d-trees. All of the RRT's nodes are additionally stored in a k-d-tree to enable a fast nearest neighbor and radius search for constructing the tree and updating nodes in a radius.

### CollisionChecker

The CollisonChecker subscribes to an occupancy grid map topic and checks if a new node can be connected to the existing tree. Therefore, a circular area at the new node and a rectangular area between the new node and the nearest neighbor in the tree are checked for obstacles in the grid map. If they are obstacle free, the node can be added, if not it is discarded.

### TreePathCalculator

The TreePathCalculator monitors the robot's position in the map, so that the node closest to the robot is always known. When the robot moves from one node to the next, the TreePathCalculator updates the path from each particular node to the robot that is maintained in the node. This path can be retrieved for navigation.

### GainCalculator

The GainCalculator calculates each node's gain by sparse ray sampling. Therefore, sampling points are pre-calculated on initialization which are defined by parameters set by the user. The gain is the number of unknown sample points that can be observed from the node. It also sets the nodes' height by obtaining the ground's height from raytracing in the OctoMap. Nodes to be updated are subscribed to and updated nodes are published.

### RneVisualizer

The RneVisualizer subscribes to the published RRT and visualizes it for RViz, showing nodes as spheres colored in the particular node's state and edges as lines between the nodes. It also publishes text visualization that shows the nodes' number and gain. This is only recommended for small trees, otherwise RViz can crash when rendering too many text elements.

## Nodes

### RneNode

Controls the RRT construction and offers interfaces to it.

#### Published Topics

**rrt_tree** ([rrt_nbv_exploration_msgs/Tree](../rrt_nbv_exploration_msgs/msg/Tree.msg))  
Current RRT tree with list of all nodes

**node_to_update** ([rrt_nbv_exploration_msgs/NodeToUpdate](../rrt_nbv_exploration_msgs/msg/NodeToUpdate.msg))  
Node which gain should be calculated and a flag if it must be recalculated

**bestAndCurrentGoal** ([rrt_nbv_exploration_msgs/BestAndCurrentNode](../rrt_nbv_exploration_msgs/msg/BestAndCurrentNode.msg))  
Current exploration goal and node with best gain-cost-ratio

**collision_visualization** ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))  
Obstacle free areas around nodes and edges checked by the RRT

#### Subscribed Topics

**<octomap_topic>** ([octomap_msgs/Octomap](http://docs.ros.org/en/melodic/api/octomap_msgs/html/msg/Octomap.html))  
OctoMap voxel grid for sampling space dimensions

**updated_node** ([rrt_nbv_exploration_msgs/Node](../rrt_nbv_exploration_msgs/msg/Node.msg))  
Node which gain was recently calculated

**<occupancy_grid_topic>** ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))  
Grid map to check for traversability

#### Services

**requestGoal** ([rrt_nbv_exploration_msgs/RequestGoal](../rrt_nbv_exploration_msgs/srv/RequestGoal.srv))  
Returns the current goal's position and orientation if available

**requestPath** ([rrt_nbv_exploration_msgs/RequestPath](../rrt_nbv_exploration_msgs/srv/RequestPath.srv))  
Returns the path from the robot to the current goal

**updateCurrentGoal** ([rrt_nbv_exploration_msgs/UpdateCurrentGoal](../rrt_nbv_exploration_msgs/srv/UpdateCurrentGoal.srv))  
Sends the status of the current goal

**setRrtState** ([std_srvs/SetBool](http://docs.ros.org/api/std_srvs/html/srv/SetBool.html))  
Start or stop exploration (true to start, false to stop)

**getRrtState** ([std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html))  
Returns if the exploration is currently running

**resetRrtState** ([std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html))  
Stops the exploration and resets the tree

#### Parameters

**~update_frequency** (float, default: 20)  
Update rate in Hz

**~sensor_max_range** (float, default: 5.0)  
Max sensor range

**~sensor_height** (float, default: 0.5)  
Height of the sensor measured from the robot's base frame (obtain with `rosrun tf_echo robot_frame sensor_frame` if unsure)

**~edge_length** (float, default: 1.0)  
Length of the edges between the RRT's nodes, variable edge length when set to 0

**~robot_radius** (float, default: 1.0)  
Circumference of the robot which must be obstacle free so that it can turn on the spot (add margin if required)

**~robot_width** (float, default: 1.0)  
Width of the robot (add margin if required)

**~exploration_finished_timer_duration** (float, default: 1.0)  
Time to wait after the last node was added to the RRT and no tree is currently the goal before stopping the exploration

**~octomap_topic** (string, default: "octomap_binary")  
Topic name of the OctoMap required for gain calculation

**~occupancy_grid_topic** (string, default: "map")  
Topic name of the occupancy grid required sampling space dimensions

**~robot_frame** (string, default: "base_footprint")  
Frame name of the robot's base frame

**~visualize_collision** (bool, default: false)  
If the collision checking should be visualized

**~check_init_position** (bool, default: false)  
If the position at which the robot currently is when starting RNE should be checked for collision

**~grid_map_resolution** (double, default: 0.05)  
Resolution of the grid map for collision checking

**~local_sampling_radius** (double, default: 5.0)  
Samples additional nodes in the given radius around the robot if radius is greater than zero

#### Required tf Transforms

**map -> <robot_frame>**  
Usually provided by SLAM

### GainCalcNode

Calculates the gain of given nodes by sparse ray sampling in the OctoMap and the height of the node by raytracing to find the ground. It places the node at sensor height above the ground.

#### Published Topics

**raysample_visualization** ([visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html))  
Visualization of the previous sparse ray sampling

**updated_node** ([rrt_nbv_exploration_msgs/Node](../rrt_nbv_exploration_msgs/msg/Node.msg))  
Node which gain was recently calculated

#### Subscribed Topics

**<octomap_topic>** ([octomap_msgs/Octomap](http://docs.ros.org/en/melodic/api/octomap_msgs/html/msg/Octomap.html))  
OctoMap voxel grid for gain calculation

**node_to_update** ([rrt_nbv_exploration_msgs/NodeToUpdate](../rrt_nbv_exploration_msgs/msg/NodeToUpdate.msg))  
Node which gain should be calculated and a flag if it must be recalculated

#### Parameters

**~sensor_max_range** (float, default: 5.0)  
Max sensor range

**~sensor_min_range** (float, default: 1.0)  
Min sensor range

**~sensor_height** (float, default: 0.5)  
Height of the sensor measured from the robot's base frame (obtain with `rosrun tf_echo robot_frame sensor_frame` if unsure)

**~sensor_size** (float, default: 0.1)  
Radius in m of a sphere that best describes the sensor's volume (checking for obstacles begins outside of this radius)

**~delta_phi** (int, default: 10)  
Step size in Phi-direction in degrees (corresponds to vertical FoV)

**~delta_theta** (int, default: 10)  
Step size in Theta-direction in degrees (corresponds to circle's circumference)

**~delta_radius** (float, default: 0.1)  
Step size in radius-direction in m

**~sensor_horizontal_fov** (int, default: 360)  
Horizontal field of view for the sensor in degrees

**~sensor_vertical_fov_bottom** (int, default: 0)  
Sensor's vertical FoV bottom edge that is considered for gain calculation (in degrees, from 180 to 0 as the highest angle)

**~sensor_vertical_fov_top** (int, default: 180)  
Sensor's vertical FoV top edge that is considered for gain calculation (in degrees, from 180 to 0 as the highest angle)

**~min_view_score** (float, default: 1.0)  
Min percentage of max possible gain for a node to be considered as a goal, if the node's gain is below this threshold, it is set to the *explored* state (see [here](../rrt_nbv_exploration_plugins#starting-rne) for additional details)

**~octomap_topic** (string, default: "octomap_binary")  
Topic name of the OctoMap required for gain calculation

**~oc_resolution** (float, default: 0.1)  
Resolution of octomap (edge length of voxels in m)

**~visualize_gain_calculation** (bool, default: false)  
If the gain calculation should be visualized

### RneVisualizationNode

Visualizes the RRT for RViz by publishing it as markers (nodes as sphere and edges as lines). The nodes are colored corresponding to their particular state (no gain: white, initial: blue, active: yellow, visited: light green, active visited: orange, explored: dark green, failed: red).

#### Published Topics

**rrt_tree_vis** ([visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html))  
Visualization of the RRT with nodes as spheres and edges as lines

**rrt_tree_vis_info** ([visualization_msgs/MarkerArray](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/MarkerArray.html))  
Visualization of the RRT's node's numbers plus gain as text

#### Subscribed Topics

**rrt_tree** ([rrt_nbv_exploration_msgs/Tree](../rrt_nbv_exploration_msgs/msg/Tree.msg))  
Current RRT tree with list of all nodes
