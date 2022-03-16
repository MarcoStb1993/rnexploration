#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include <rrg_nbv_exploration_msgs/Graph.h>
#include <rrg_nbv_exploration_msgs/Node.h>
#include <rrg_nbv_exploration_msgs/Edge.h>
#include <rrg_nbv_exploration_msgs/NodeToUpdate.h>
#include <rrg_nbv_exploration_msgs/BestAndCurrentNode.h>
#include <rrg_nbv_exploration_msgs/RequestGoal.h>
#include <rrg_nbv_exploration_msgs/RequestPath.h>
#include <rrg_nbv_exploration_msgs/UpdateCurrentGoal.h>
#include <limits.h>
#include <random>
#include "math.h"

#include <rrg_nbv_exploration/CollisionChecker.h>
#include <rrg_nbv_exploration/NodeComparator.h>
#include <rrg_nbv_exploration/GlobalGraphHandler.h>

#define SQRT10 sqrt(10.0)

namespace rrg_nbv_exploration {

/**
 * @brief The GraphConstructor class constructs a RRG at a given seed position and publishes it in the rrg topic
 */
class GraphConstructor {
public:
	/**
	 * @brief Constructor that places the graph's seed at (0,0,0)
	 */
	GraphConstructor();
	~GraphConstructor();
	/**
	 * @brief Initialization for ROS and graph GraphConstructor, called by the constructors
	 * @param Seed position for the tree
	 */
	void initialization(geometry_msgs::Point seed = geometry_msgs::Point());
	/**
	 * @brief Starts the tree GraphConstructor when stopped
	 */
	void startRrgConstruction();
	/**
	 * @brief Stops the GraphConstructor when running
	 */
	void stopRrgConstruction();
	/**
	 * @brief GraphConstructor main function, publishes it in topic rrg
	 * @param kd-tree for faster nearest neighbour queries
	 */
	void runRrgConstruction();

	void dynamicReconfigureCallback(
			rrg_nbv_exploration::GraphConstructorConfig &config,
			uint32_t level);

private:
	ros::NodeHandle _nh;

	ros::Publisher _rrg_publisher;
	ros::Publisher _best_and_current_goal_publisher;
	ros::Publisher _node_to_update_publisher;
	ros::Subscriber _updated_node_subscriber;
	ros::Subscriber _octomap_sub;
	ros::ServiceServer _request_goal_service;
	ros::ServiceServer _request_path_service;
	ros::ServiceServer _update_current_goal_service;
	ros::ServiceServer _set_rrg_state_service;
	ros::ServiceServer _get_rrg_state_service;
	ros::ServiceServer _reset_rrg_state_service;
	ros::Timer _exploration_finished_timer;

	std::default_random_engine _generator;
	std::shared_ptr<octomap::AbstractOcTree> _abstract_octree;
	std::shared_ptr<octomap::OcTree> _octree;
	/**
	 * @brief Helper class for checking if a path between two nodes is collision free
	 */
	std::shared_ptr<CollisionChecker> _collision_checker;
	/**
	 * @brief Helper class for radius and nearest neighbor search in kd-tree based on RRG
	 */
	std::shared_ptr<GraphSearcher> _graph_searcher;
	/**
	 * @brief Helper class for calculating a path between two nodes in the graph
	 */
	std::shared_ptr<GraphPathCalculator> _graph_path_calculator;
	/**
	 * @brief Helper class for calculating the gain cost ratio of a node and sorting them ordered by this ratio
	 */
	std::shared_ptr<NodeComparator> _node_comparator;
	/**
	 * @brief Helper class for managing the global graph with its frontiers and paths connecting them and the local graph
	 */
	std::shared_ptr<GlobalGraphHandler> _global_graph_handler;
	;
	/**
	 * @brief Current graph being built as a RRG
	 */
	rrg_nbv_exploration_msgs::Graph _rrg;
	/**
	 * @brief State of the GraphConstructor (running or stopped)
	 */
	bool _running;
	/**
	 * @brief Current min value of bounding box of the 3D map as x, y and z value
	 */
	double _map_min_bounding[3] = { 0, 0, 0 };
	/**
	 * @brief Current max value of bounding box of the 3D map as x, y and z value
	 */
	double _map_max_bounding[3] = { 0, 0, 0 };
	/**
	 * @brief List of nodes (their position in the graph's node list) that require their gain to be calculated
	 */
	std::list<int> _nodes_to_update;
	/**
	 * @brief The node that is currently being pursued as a navigation goal
	 */
	int _current_goal_node;
	/**
	 * @brief Previously visited goal node
	 */
	int _last_goal_node;
	/**
	 * @brief Maximal sensor range that is considered for gain calculation in m
	 */
	double _sensor_range;
	/**
	 * @brief Doubled and squared sensor range for radius search in kd-tree
	 */
	double _radius_search_range;
	/**
	 * @brief Radius of the RRG around the robot in m
	 */
	double _local_graph_radius;
	/**
	 * @brief Squared radius of the RRG around the robot in m
	 */
	double _local_graph_radius_squared;
	/**
	 * @brief Distance on z-axis between base footprint and sensor frame
	 */
	double _sensor_height;
	/**
	 * @brief Min distance between two nodes in the graph
	 */
	double _min_edge_distance;
	/**
	 * @brief Max distance between two nodes in the graph
	 */
	double _max_edge_distance;
	/**
	 * @brief Squared max distance between two nodes in the graph
	 */
	double _max_edge_distance_squared;
	/**
	 * @brief Previous recorded robot position
	 */
	geometry_msgs::Point _last_robot_pos;
	/**
	 * @brief Index of previously updated node
	 */
	int _last_updated_node;
	/**
	 * @brief Time until exploration counts as finished because no new nodes were placed
	 */
	double _exploration_finished_timer_duration;
	/**
	 * @brief If the robot already moved past a neighbor node towards the current goal
	 */
	bool _moved_to_current_goal;
	/**
	 * @brief Indicator if current goal was already updated before a new goal can be requested
	 */
	bool _goal_updated;
	/**
	 * @brief Radius that includes robot's footprint in m
	 */
	double _robot_radius;
	/**
	 * @brief Squared radius that includes robot's footprint in m
	 */
	double _robot_radius_squared;
	/**
	 * @brief If current goal is currently updating
	 */
	bool _updating;
	/**
	 * @brief If sorting the list of nodes to update is required
	 */
	bool _sort_nodes_to_update;
	/**
	 * @brief Grid map cell edge length in m
	 */
	double _grid_map_resolution;
	/**
	 * @brief If > 0 samples additional nodes in the given radius around the robot
	 */
	double _local_sampling_radius;
	/**
	 * @brief Maximum number of consecutive failed goals before exploration is cancelled
	 */
	int _max_consecutive_failed_goals;
	/**
	 * @brief Actual number of consecutive failed goals
	 */
	int _consecutive_failed_goals;
	/**
	 * @brief Current pose of the robot
	 */
	geometry_msgs::Pose _robot_pose;
	/**
	 * @brief List of nodes where navigation failed which will be tried to recover when the next goal (or movement to it) was successful
	 */
	std::list<int> _failed_nodes_to_recover;
	/**
	 * @brief Return to the root node if all nodes were either explored or failed
	 */
	bool _auto_homing;
	/**
	 * @brief If active nodes' gain should be recalculated when the robot's nearest neighbors changes
	 */
	bool _reupdate_nodes;
	/**
	 * @brief Keep track of the last three unique nearest nodes on the path the robot took
	 */
	std::vector<int> _last_three_nodes_path;
	/**
	 * @brief List of nodes ordered ascending by distance to robot which gain can be recalculated because the robot moved
	 */
	std::vector<int> _nodes_to_reupdate;
	/**
	 * @brief Index of the next node in the path from the nearest node to the robot to the current goal
	 */
	int _next_node;
	/**
	 * @brief Index of the edge to the next node in the path from the nearest node to the robot to the current goal
	 */
	int _edge_to_next_node;
	/**
	 * @brief Squared distance to the currently nearest node in m
	 */
	double _distance_to_nearest_node_squared;
	/**
	 * @brief If the current goal node was set to explored by an update
	 */
	bool _explored_current_goal_node_by_update;
	/**
	 * @brief The number of attempted samples each loop
	 */
	int _samples_per_loop;

	/**
	 * @brief Initialize the RRG with a root node at seed, initialize helper classes and nodes ordered by gain list with root node
	 * @param Seed position for RRG at which the root node is placed
	 */
	void initRrg(const geometry_msgs::Point &seed);
	/**
	 * @brief Samples new nodes and tries to connect them to the graph
	 * @param If the paths of possibly connected nodes should be updated
	 */
	void expandGraph(bool update_paths);

	/**
	 * @brief Removes nodes that are outside the sliding RRG radius around the robot including
	 * nodes that were disconnected from the RRG and their respective edges and adds nodes with
	 * gain to the global graph as well as continuing paths in the global graph
	 */
	void pruneLocalGraph();
	/**
	 * @brief Randomly samples a point from within the minimum of the sensor range and RRG radius
	 * around the robot
	 * @param Reference to a point that is filled with randomly sampled x and y coordinates
	 * @return If the point is inside map dimensions and was successfully created
	 */
	bool samplePoint(geometry_msgs::Point &rand_sample);
	/**
	 * @brief Randomly samples a point within the minimum of the local sampling range and RRG radius
	 * around the robot
	 * @param Reference to a point that is filled with randomly sampled x and y coordinates
	 * @return If the point is inside map dimensions and was successfully created
	 */
	bool samplePointLocally(geometry_msgs::Point &rand_sample);
	/**
	 * @brief Randomly samples a point from within a circle with the provided radius around the given center
	 * @param Reference to a point that is filled with randomly sampled x and y coordinates
	 * @param Center for the sampling area
	 * @param Radius of the sampling area
	 * @return If the point is inside map dimensions and was successfully created
	 */
	bool samplePointInRadius(geometry_msgs::Point &rand_sample,
			geometry_msgs::Point center, double radius);

	/**
	 * @brief Check if there is a current goal, if there are still nodes to be explored and select a new goal if required and possible
	 */
	void checkCurrentGoal();
	/**
	 * @brief Check if a new node is nearest to the robot and if it is on the path to the current goal
	 * @param Robot position
	 * @return Returns true if a new node is nearest to the robot
	 */
	bool determineNearestNodeToRobot(geometry_msgs::Point robot_pos);
	/**
	 * @brief Publish the node that currently has the best gain-cost-ratio
	 */
	void publishBestAndCurrentNode();
	/**
	 * @brief Sorts the nodes which gain needs to be (re)calculated by their distance to the robot (closest first)
	 */
	void sortNodesToUpdateByDistanceToRobot();
	/**
	 * @brief Compares the distances to the robot of two nodes
	 * @param Node index of the first node
	 * @param Node index of the second node
	 * @return Returns true if the first node's distance to the robot is smaller than the second
	 */
	bool compareNodeDistancesToRobot(const int &node_one, const int &node_two);
	/**
	 * @brief Publish a node which gain needs to be (re)calculated
	 */
	void publishNodeToUpdate();
	/**
	 * @brief Updates all nodes' gains in the doubled sensor range around the specified center node
	 * @param Node in the center of the update circle
	 */
	void updateNodes(geometry_msgs::Point center_node);
	/**
	 * @brief Callback for subscriber to "updated_node" topic which refreshes the list of nodes to update
	 * @param Node which gain was updated
	 */
	void updatedNodeCallback(
			const rrg_nbv_exploration_msgs::Node::ConstPtr &updated_node);
	/**
	 * @brief Function called by subscriber to "octomap_binary" message and converts it to the octree data format for further processing
	 * @param "octomap_binary" message
	 */
	void convertOctomapMsgToOctree(
			const octomap_msgs::Octomap::ConstPtr &map_msg);
	/**
	 * @brief Updates the map's dimension (x,y,z) after a new octomap was received
	 */
	void updateMapDimensions();
	/**
	 * @brief Updates the current goal and the goals around it depending on the status navigation returned
	 */
	void handleCurrentGoalFinished();

	/**
	 * @brief Try to recover failed nodes by repeating the collision check and queue them up for gain
	 * calculation if the former was successful, set them to initial if recovery succeeded
	 * Also tries to recover failed edges if they had a collision with unknown tiles (non-inflation only)
	 */
	void tryFailedNodesRecovery();

	/**
	 * @brief Adds the given node to the list of last three nodes in the path of the robot with removal
	 * of duplicate nodes and the removal of the oldest node if the size limit (3) is reached which
	 * also causes the list of nodes to re-update to be refreshed if a new node was added
	 * @param Node index to add
	 */
	void addNodeToLastThreeNodesPath(int node);

	/**
	 * @brief Determine the next node in the path to the current goal from the node nearest to the robot
	 * as well as the edge connecting the nearest and the next node
	 */
	void determineNextNodeInPath();

	/**
	 * @brief Set next node in the path to the current goal from the node nearest to the robot and the
	 * edge connecting next and nearest node to -1
	 */
	void resetNextNodeInPath();

	/**
	 * @brief Iterates over all nodes and sets them to inactive if they are outside the robot's sensor
	 * range, also returns a set of all disconnected nodes that had a deactivated node in their path to
	 * the robot
	 * @param Reference to the set of deactivated nodes
	 * @param Reference to the set of deactivated edges
	 * @return List of nodes that had one of the deactivated nodes in their path to the robot
	 */
	std::vector<int> findOutOfSensorRadiusNodes(std::set<int> &pruned_nodes,
			std::set<int> &pruned_edges);

	/**
	 * @brief Checks if the given node is next in the path to the robot of the neighbor node
	 * @param Index of the neighbor node
	 * @param Index of the node
	 * @return If the given node is next in the path to the robot of the neighbor node
	 */
	bool isNextInPathToRobot(int neighbor_node, int node);

	/**
	 * @brief Sets the given node to inactive and removes it from all update lists
	 * @param Index of the node to deactivate
	 */
	void deactivateNode(int node);

	/**
	 * @brief Compare two nodes and return if the path to the robot of the first is smaller than the second
	 * @param First node index
	 * @param Second node index
	 * @return If the first node's path to the robot is smaller than the second's
	 */
	bool sortByPathLength(int node_one, int node_two);

	/**
	 * @brief Removes the reference to this edge from the remaining node at the other end of the
	 * given edge from the provided node
	 * @param Index of the edge connecting the nodes
	 * @param Index of the node that is deactivated
	 */
	void removeEdgeFromRemainingNode(int edge, int node);

	/**
	 * @brief Removes the pruned edges or adds them to available edges depending on the index
	 * @param Reference to the set of pruned edges
	 */
	void handlePrunedEdges(const std::set<int> &pruned_edges);

	/**
	 * @brief Removes the pruned nodes or adds them to available nodes depending on the index
	 * @param Reference to the set of pruned nodes
	 */
	void handlePrunedNodes(const std::set<int> &pruned_nodes);

	/**
	 * @brief Remove given node from node comparator, nodes to update and nodes to re-update lists
	 * @param Node index
	 */
	void removeNodeFromUpdateLists(int node);

	/**
	 * @brief Find connected nodes with gain and cluster them to only allow of of the connected nodes
	 * to become a frontier (by setting the other node's gain to 0)
	 * @param List of pruned nodes ordered ascending by path length to robot
	 */
	void findConnectedNodesWithGain(std::vector<int> &nodes);

	/**
	 * @brief Iterates over the vector of nodes that were disconnected because a deactivated node was
	 * in their path to the robot and adds further disconnected nodes as well as finding nodes with an
	 * edge to a disconnected node and a viable path to the robot
	 * @param Reference to the list of disconnected nodes
	 * @param Reference to the set of nodes still connected to a disconnected node and the remaining RRG
	 */
	void findAllConnectedAndDisconnectedNodes(
			std::vector<int> &disconnected_nodes,
			std::set<int> &connected_nodes);

	/**
	 * Timer callback for setting exploration to finished
	 * @param event
	 */
	void explorationFinishedTimerCallback(const ros::TimerEvent &event);

	bool requestGoal(rrg_nbv_exploration_msgs::RequestGoal::Request &req,
			rrg_nbv_exploration_msgs::RequestGoal::Response &res);

	bool requestPath(rrg_nbv_exploration_msgs::RequestPath::Request &req,
			rrg_nbv_exploration_msgs::RequestPath::Response &res);

	bool updateCurrentGoal(
			rrg_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
			rrg_nbv_exploration_msgs::UpdateCurrentGoal::Response &res);

	bool setRrgState(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);

	bool getRrgState(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);

	bool resetRrgState(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
};
}
