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
	 * @brief Helper class for kd-tree radius and nearest neighbor search
	 */
	std::shared_ptr<GlobalGraphSearcher> _global_graph_searcher;
	/**
	 * @brief Helper class for radius and nearest neighbor search in kd-tree based on RRG
	 */
	std::shared_ptr<GraphSearcher> _graph_searcher;
	/**
	 * @brief Helper class for checking if a path between two nodes is collision free
	 */
	std::shared_ptr<CollisionChecker> _collision_checker;

	/**
	 * @brief Maximal sensor range that is considered for gain calculation in m
	 */
	double _sensor_range;
	/**
	 * @brief Squared maximal sensor range that is considered for gain calculation in m
	 */
	double _sensor_range_squared;
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
	 * by the given index or by adding it to the list if none is available
	 * @param Reference of the frontier to be inserted
	 */
	void insertFrontierInGg(
			const rrg_nbv_exploration_msgs::GlobalFrontier &frontier);

	/**
	 * @brief Add the given path to the global graph by inserting it at an available position marked
	 * by the given index or by adding it to the list if none is available
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
	void connectFrontiers(int frontier_one, int frontier_two,
			rrg_nbv_exploration_msgs::GlobalPath &path, int path_at_node,
			bool connecting_node);
	bool mergeNeighborFrontiers(int node,
			const rrg_nbv_exploration_msgs::GlobalPath &path,
			rrg_nbv_exploration_msgs::Graph &rrg,
			rrg_nbv_exploration_msgs::GlobalFrontier &frontier);
};

} /* namespace rrg_nbv_exploration */

#endif /* RRG_NBV_EXPLORATION_SRC_GLOBALGRAPHHANDLER_H_ */
