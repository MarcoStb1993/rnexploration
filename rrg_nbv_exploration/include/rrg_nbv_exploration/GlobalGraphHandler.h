/*
 * GlobalGraphHandler.h
 *
 *  Created on: Mar 4, 2022
 *      Author: marco
 */

#ifndef RRG_NBV_EXPLORATION_SRC_GLOBALGRAPHHANDLER_H_
#define RRG_NBV_EXPLORATION_SRC_GLOBALGRAPHHANDLER_H_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <rrg_nbv_exploration_msgs/GlobalGraph.h>
#include <rrg_nbv_exploration_msgs/Graph.h>
#include <rrg_nbv_exploration/GraphPathCalculator.h>
#include <rrg_nbv_exploration/GlobalGraphSearcher.h>
#include <rrg_nbv_exploration/GlobalPathWaypointSearcher.h>
#include <rrg_nbv_exploration/GraphSearcher.h>
#include <rrg_nbv_exploration/CollisionChecker.h>

namespace rrg_nbv_exploration {

/**
 * @brief The GlobalGraphHandler class constructs a global graph which connects
 */
class GlobalGraphHandler {
public:
	GlobalGraphHandler();
	virtual ~GlobalGraphHandler();

	/**
	 * @brief Initialize the global graph with the origin at the local graph's root node as the first
	 * frontier and the corresponding path
	 * @param The local graph's root node
	 * @param Helper class for path calculation in the RRG
	 * @param Helper class for radius and nearest neighbor search in RRG
	 * @param Helper class for collision checks on traversability map
	 */
	void initialize(rrg_nbv_exploration_msgs::Node &root,
			std::shared_ptr<GraphPathCalculator> graph_path_calculator,
			std::shared_ptr<GraphSearcher> graph_searcher,
			std::shared_ptr<CollisionChecker> collision_checker);

	/**
	 * @brief Publishes the global graph message on the "globalgraph" topic
	 */
	void publishGlobalGraph();

	/**
	 * @brief Adds the given node from the local graph as a new frontier and connect it to the
	 * local graph and other frontiers by a path
	 * @param Index of the node that will become a frontier
	 * @param Reference to the RRG
	 */
	void addFrontier(int node, rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Continues the path to a frontier for the given node that it was connected to and connects
	 * to other frontiers with a new path if applicable
	 * @param Index of the node that was removed from the local graph and had at least one connection to a path
	 * @param Reference to the RRG
	 */
	void continuePath(int node, rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Checks for all frontiers in the max sensor range around the robot if they can be connected
	 * to a node in the local graph and deactivates them and their respective paths
	 * @param Reference to the RRG
	 * @param Current robot position
	 */
	void deactivateFrontiersInLocalGraph(rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Point robot_position);

	/**
	 * @brief Checks if a new node in the RRG can reduce the global paths, which have a connection to
	 * a neighbor node of the new node, if their nearest respective waypoint would be connected to
	 * the new node
	 * @param Reference to the RRG
	 * @param Index of the new node
	 */
	void checkPathsWaypoints(rrg_nbv_exploration_msgs::Graph &rrg,
			int new_node);

	/**
	 * @brief Calculates and stores the frontier to be explored next when the local graph has no more goals
	 * @param Reference to the RRG
	 * @return If a frontier goal could be calculated, false if no frontier available, exploration finished
	 */
	bool calculateNextFrontierGoal(rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Gets the position of the next frontier and the yaw orientation derived from the incoming
	 * waypoint or the robot position if there is only one waypoint in the path to this frontier
	 * @param Reference to the frontier position which will be inserted
	 * @param Reference to the desired yaw at the frontier position which will be calculated
	 * @param Reference to the robot position
	 * @return If a valid next frontier and path to this frontier are available
	 */
	bool getFrontierGoal(geometry_msgs::Point &goal, double &yaw,
			geometry_msgs::Point &robot_pos);

	/**
	 * @brief Get the waypoints from the path to the next frontier and the node in the local graph
	 * connected to the path and return a navigation path along those waypoints
	 * @param Reference to the navigation path which will be filled
	 * @param Reference to robot position
	 * @return If a valid next frontier and path to this frontier are available
	 */
	bool getFrontierPath(std::vector<geometry_msgs::PoseStamped> &path,
			geometry_msgs::Point &robot_pos);

	/**
	 * @brief Deactivate next frontier and its paths from global graph and use the frontier as the new root
	 * location for the RRG and its paths to other frontiers as new paths to the local graph for the
	 * particular frontiers
	 * @param Reference to the frontier's viewpoint which will be inserted
	 * @param Returns a list of all paths which are connected to the new local graph's root node
	 */
	std::vector<int> frontierReached(geometry_msgs::Point &position);

	/**
	 * @brief Updates the waypoint in the active path that is closest to the robot's current position
	 * @param Reference to the robot position
	 * @return Return if the frontier is the closest waypoint
	 */
	bool updateClosestWaypoint(geometry_msgs::Point &robot_pos);

	void dynamicReconfigureCallback(
			rrg_nbv_exploration::GraphConstructorConfig &config,
			uint32_t level);

private:

	ros::NodeHandle _nh;
	ros::Publisher _global_graph_publisher;

	/**
	 * @brief Global graph object with frontiers and paths
	 */
	rrg_nbv_exploration_msgs::GlobalGraph _gg;
	/**
	 * @brief Helper class for calculating path, heading and traversability between any two nodes in
	 * the graph
	 */
	std::shared_ptr<GraphPathCalculator> _graph_path_calculator;
	/**
	 * @brief Helper class for kd-tree radius and nearest neighbor search in global graph
	 */
	std::shared_ptr<GlobalGraphSearcher> _global_graph_searcher;
	/**
	 * @brief Helper class for kd-tree radius and nearest neighbor search in a path's waypoints
	 */
	std::shared_ptr<GlobalPathWaypointSearcher> _global_path_waypoint_searcher;
	/**
	 * @brief Helper class for radius and nearest neighbor search in kd-tree based on RRG
	 */
	std::shared_ptr<GraphSearcher> _graph_searcher;
	/**
	 * @brief Helper class for checking if a path between two nodes is collision free
	 */
	std::shared_ptr<CollisionChecker> _collision_checker;

	/**
	 * @brief If the inflation of nodes (wavefront) is active
	 */
	bool _inflation_active;
	/**
	 * Radius that includes robot's footprint in m
	 */
	double _robot_radius;
	/**
	 * Squared radius that includes robot's footprint in m
	 */
	double _robot_radius_squared;
	/**
	 * Width of the robot in m
	 */
	double _robot_width;
	/**
	 * Minimum required distance between two nodes for a box collision object to be inserted
	 */
	double _path_box_distance_thres;
	/**
	 * @brief Squared max distance between two nodes in the graph
	 */
	double _max_edge_distance_squared;
	/**
	 * @brief Ordered list of frontier indices where the frontiers are inactive and can be replaced with a new frontier
	 */
	std::set<int> _available_frontiers;
	/**
	 * @set Ordered list of path indices where the paths are inactive and can be replaced with a new path
	 */
	std::set<int> _available_paths;
	/**
	 * @brief Radius of the RRG around the robot in m
	 */
	double _local_graph_radius;
	/**
	 * @brief Squared radius of the RRG around the robot in m
	 */
	double _local_graph_radius_squared;
	/**
	 * @brief Index of the frontier (first) to be explored next with its path to follow (second)
	 */
	std::pair<int, int> _next_frontier_with_path;
	/**
	 * @brief While global navigation is active, holds the index of the followed path and the closest waypoint
	 */
	std::pair<int, int> _active_paths_closest_waypoint;
	/**
	 * @brief Return to the origin node when all nodes frontiers were explored
	 */
	bool _auto_homing;

	/**
	 * @brief Iterates through the given node's path to the robot in the local graph and adds all nodes
	 * as waypoints that were deactivated, the first active node is set as the connecting node for the
	 * given path
	 * @param Index of the node at which to start
	 * @param Reference to the RRG
	 * @param Reference to the list of waypoints
	 * @param Reference to the connecting node for a path
	 * @param Reference to the length of the connecting path
	 * @return If a connection to an active node of the local graph could be made from this node
	 */
	bool getConnectingNode(int node, rrg_nbv_exploration_msgs::Graph &rrg,
			std::vector<geometry_msgs::Point> &waypoints, int &connecting_node,
			double &length);

	/**
	 * @brief Retrieve the lowest available frontier index for a new frontier (uses inactive frontiers if available)
	 * @return Frontier index for new frontier
	 */
	int availableFrontierIndex();

	/**
	 * @brief Retrieve the lowest available path index for a new path (uses inactive paths if available)
	 * @return Path index for new path
	 */
	int availablePathIndex();

	/**
	 * @brief Deactivate all pruned frontiers and remove them from the list of frontiers if they are at the end
	 * @param Reference to the set of pruned frontiers
	 */
	void handlePrunedFrontiers(const std::set<int> &pruned_frontiers);

	/**
	 * @brief Deactivate all pruned paths and remove them from the list of paths if they are at the end
	 * @param Reference to the set of pruned paths
	 */
	void handlePrunedPaths(const std::set<int> &pruned_paths);

	/**
	 * @brief Deactivate the frontier with the given index by setting it to inactive and removing its paths
	 * @param Index of frontier to deactivate
	 */
	void deactivateFrontier(int pruned_frontier);

	/**
	 * @brief Deactivate the path with the given index by setting it to inactive and removing its waypoints
	 * @param Index of path to deactivate
	 */
	void deactivatePath(int pruned_path);

	/**
	 * @brief Add the given frontier to the global graph by inserting it at an available position marked
	 * by the given index or by adding it to the end of the list if none is available
	 * @param Reference of the frontier to be inserted
	 */
	void insertFrontierInGg(
			const rrg_nbv_exploration_msgs::GlobalFrontier &frontier);

	/**
	 * @brief Add the given path to the global graph by inserting it at an available position marked
	 * by the given index or by adding it to the end of the list if none is available
	 * @param Reference of the path to be inserted
	 */
	void insertPathInGg(
			const rrg_nbv_exploration_msgs::GlobalPath &path_between_frontiers);

	/**
	 * @brief Add the given frontier index to the list of frontiers to be pruned from the global graph
	 * including all paths connected to it and remove their connection from the nodes in the RRG
	 * @param Index of the frontier to be pruned
	 * @param Reference to the set of frontiers to be pruned
	 * @param Reference to the set of paths to be pruned
	 * @param Reference to the RRG
	 */
	void addFrontierToBePruned(int frontier_to_prune,
			std::set<int> &pruned_frontiers, std::set<int> &pruned_paths,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Create a new path connecting the first and second frontier at a connecting node
	 *  and add it to the global graph with the waypoints being a union from both paths
	 * @param Index of the first frontier with a path to a node in the local graph
	 * @param Index of the second frontier with a path to a node in the local graph
	 * @param Index to the path of the first node
	 * @param Index of the path of the second node
	 * @param If the node at which both frontier's path meet is a connecting node or the deactivated
	 * node at which the first frontier should be placed
	 */
	void connectFrontiers(int frontier_one, int frontier_two, int path_one,
			int path_two, bool connecting_node);

	/**
	 * @brief Check if there are any paths to existing frontiers at the node or connecting node of a
	 * new frontier and merge them if there are, the frontier with the shortest distance to the local
	 * graph remains, all others are deactivated (including their paths)
	 * @param Index of the node in the local graph at which the new frontier should be placed
	 * @param Reference to the path of the new frontier
	 * @param Reference to the RRG
	 * @param Reference to the new frontier
	 * @return True if the new frontier can be placed in the global graph, false if it was merged into
	 * an existing frontier
	 */
	bool mergeNeighborFrontiers(int node,
			const rrg_nbv_exploration_msgs::GlobalPath &path,
			rrg_nbv_exploration_msgs::Graph &rrg,
			rrg_nbv_exploration_msgs::GlobalFrontier &frontier);

	/**
	 * @brief Determine the waypoint from the given node which is closest to the frontier of the give path
	 * and can be connected to the new node which
	 * @param Index of the path
	 * @param Index of the new node in the RRG
	 * @param List of waypoints of the given path that could potentially be connected to the new node
	 * ordered ascending by distance to the new node (first=index of the waypoint, second=squared distance)
	 * @param Reference to the RRG
	 * @return Index of a connectable waypoint closest to the frontier (defaults to last index if no
	 * suitable waypoint was found)
	 */
	int getClosestWaypointToFrontier(int path, int new_node,
			std::vector<std::pair<int, double> > &waypoints_near_new_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Calculate the resulting path length if the given path would be connected to the new node
	 * from the provided closest waypoint
	 * @param Index of the path
	 * @param Index of the new node in the RRG
	 * @param Index of the closest waypoint to the frontier of the given path that can be connected
	 * to the new node
	 * @param Reference to the RRG
	 * @return Resulting path length when rewiring the path	 *
	 */
	double calculateNewPathLength(int path, int new_node,
			int closest_waypoint_to_frontier,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Rewires the given path to the provided new node using the given waypoint
	 * @param Index of the path
	 * @param Index of the closest waypoint to the frontier of the given path that can be connected
	 * to the new node
	 * @param New path length when rewiring the path
	 * @param Index of the new node in the RRG
	 * @param Reference to the RRG
	 */
	void rewirePathToNewNode(int path, int closest_waypoint_to_frontier,
			double new_path_length, int new_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Attempts to improve existing connections between frontiers, because the frontiers from the
	 * given path was rewired to a new node in the RRG, by checking if a connection via the new node would
	 * reduce the path length and also creates new connections if a path to a frontier is found that did
	 * not exist before
	 * @param Index of the new node in the RRG
	 * @param Index of the path
	 * @param Reference to the RRG
	 */
	void tryToImproveConnectionsToOtherFrontiers(int new_node, int path,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Improves an existing connection between two frontiers because one of the frontiers
	 * was rewired to a new node in the RRG and the rewiring leads to a shorter path
	 * @param Index of the path connecting the frontiers
	 * @param Index of the path to the local graph that was rewired
	 * @param Index of the other frontier which connection will be improved
	 * @param Index of the other frontier's path to the local graph
	 */
	void improvePathToConnectedFrontier(int frontier_path, int path,
			int other_frontier, int other_path);
	bool checkIfNextFrontierWithPathIsValid();
};

} /* namespace rrg_nbv_exploration */

#endif /* RRG_NBV_EXPLORATION_SRC_GLOBALGRAPHHANDLER_H_ */
