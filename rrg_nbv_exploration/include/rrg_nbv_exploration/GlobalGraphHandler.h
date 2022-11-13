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
#include <rrg_nbv_exploration/GlobalGraphWaypointSearcher.h>
#include <rrg_nbv_exploration/GraphSearcher.h>
#include <rrg_nbv_exploration/CollisionChecker.h>

#include <stdexcept>

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
	 * target and the corresponding connection
	 * @param The local graph's root node
	 * @param Helper class for connection calculation in the RRG
	 * @param Helper class for radius and nearest neighbor search in RRG
	 * @param Helper class for collision checks on traversability map
	 */
	void initialize(rrg_nbv_exploration_msgs::Node &root,
			std::shared_ptr<GraphPathCalculator> graph_connection_calculator,
			std::shared_ptr<GraphSearcher> graph_searcher,
			std::shared_ptr<CollisionChecker> collision_checker);

	/**
	 * @brief Publishes the global graph message on the "globalgraph" topic
	 */
	void publishGlobalGraph();

	/**
	 * @brief Adds the given node from the local graph as a new target and connect it to the
	 * local graph and other targets by a connection
	 * @param Index of the node that will become a target
	 * @param Reference to the RRG
	 */
	void addFrontier(int node, rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Continues the connection to a target for the given node that it was connected to and connects
	 * to other targets with a new connection if applicable
	 * @param Index of the node that was removed from the local graph and had at least one connection to a connection
	 * @param Reference to the RRG
	 */
	void continuePath(int node, rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Checks for all targets in the max sensor range around the robot if they can be connected
	 * to a node in the local graph and deactivates them and their respective connections
	 * @param Reference to the RRG
	 * @param Current robot position
	 */
	void deactivateFrontiersInLocalGraph(rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Point robot_position);

	/**
	 * @brief Checks if a new node in the RRG can reduce the global connections, if their nearest respective
	 * waypoint would be connected to the new node. If the new node is very close to the target itself,
	 * remove the target from the global graph, if it is in a connectable range, return the target's
	 * position
	 * @param Reference to the RRG
	 * @param Index of the new node
	 * @return List of positions where to place a new node to prune targets
	 */
	std::vector<geometry_msgs::Point> pruneFrontiersAndPathsAroundNewNode(
			rrg_nbv_exploration_msgs::Graph &rrg, int new_node);

	/**
	 * @brief Calculates and stores the target to be explored next when the local graph has no more goals
	 * @param Reference to the RRG
	 * @return If a target goal could be calculated, false if no target available, exploration finished
	 */
	bool calculateNextFrontierGoal(rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Gets the position of the next target and the yaw orientation derived from the incoming
	 * waypoint or the robot position if there is only one waypoint in the connection to this target
	 * @param Reference to the target position which will be inserted
	 * @param Reference to the desired yaw at the target position which will be calculated
	 * @param Reference to the robot position
	 * @return If a valid next target and connection to this target are available
	 */
	bool getFrontierGoal(geometry_msgs::Point &goal, double &yaw,
			geometry_msgs::Point &robot_pos);

	/**
	 * @brief Get the waypoints from the connection to the next target and the node in the local graph
	 * connected to the connection and return a navigation connection along those waypoints
	 * @param Reference to the navigation connection which will be filled
	 * @param Reference to robot position
	 * @return If a valid next target and connection to this target are available
	 */
	bool getFrontierPath(std::vector<geometry_msgs::PoseStamped> &connection,
			geometry_msgs::Point &robot_pos);

	/**
	 * @brief Deactivate next target and its connections from global graph and use the target as the new root
	 * location for the RRG and its connections to other targets as new connections to the local graph for the
	 * particular targets
	 * @param Reference to the target's viewpoint which will be inserted
	 * @param Returns a list of all connections which are connected to the new local graph's root node
	 */
	std::vector<int> targetReached(geometry_msgs::Point &position);

	/**
	 * @brief Initiates navigation to the next target if the current target goal is not the last
	 * in the global route
	 * @return If the exploration is finished because no more targets are available
	 */
	bool targetFailed();

	/**
	 * @brief Updates the waypoint in the active connection that is closest to the robot's current position
	 * @param Reference to the robot position
	 * @return Return if the target is the closest waypoint
	 */
	bool updateClosestWaypoint(geometry_msgs::Point &robot_pos);

	/**
	 * @brief Checks if the next target and the respective connection exist
	 * @return If the next target and connection are valid
	 */
	bool checkIfNextFrontierWithPathIsValid();

	void dynamicReconfigureCallback(
			rrg_nbv_exploration::GraphConstructorConfig &config,
			uint32_t level);

private:

	/**
	 * @brief Structure to store information crucial to decide which target will be pruned and which
	 * remains when merging targets
	 */
	struct MergeableFrontierStruct {
		int target;
		double connection_length_to_local_graph;
		double distance_between_targets;

		MergeableFrontierStruct(int f, double connection, double distance) {
			target = f;
			connection_length_to_local_graph = connection;
			distance_between_targets = distance;
		}
	};

	ros::NodeHandle _nh;
	ros::Publisher _global_graph_publisher;

	/**
	 * @brief Global graph object with targets and connections
	 */
	rrg_nbv_exploration_msgs::GlobalGraph _gg;
	/**
	 * @brief Helper class for calculating connection, heading and traversability between any two nodes in
	 * the graph
	 */
	std::shared_ptr<GraphPathCalculator> _graph_connection_calculator;
	/**
	 * @brief Helper class for kd-tree radius and nearest neighbor search in global graph
	 */
	std::shared_ptr<GlobalGraphSearcher> _global_graph_searcher;
	/**
	 * @brief Helper class for kd-tree radius and nearest neighbor search in a connection's waypoints
	 */
	std::shared_ptr<GlobalGraphWaypointSearcher> _global_connection_waypoint_searcher;
	/**
	 * @brief Helper class for radius and nearest neighbor search in kd-tree based on RRG
	 */
	std::shared_ptr<GraphSearcher> _graph_searcher;
	/**
	 * @brief Helper class for checking if a connection between two nodes is collision free
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
	 * Half the minimum required distance between two nodes for a box collision object to be inserted
	 */
	double _half_connection_box_distance_thres;
	/**
	 * @brief Squared max distance between two nodes in the graph
	 */
	double _max_edge_distance_squared;
	/**
	 * @brief Squared min distance between two nodes in the graph
	 */
	double _min_edge_distance_squared;
	/**
	 * @brief Ordered list of target indices where the targets are inactive and can be replaced with a new target
	 */
	std::set<int> _available_targets;
	/**
	 * @set Ordered list of connection indices where the connections are inactive and can be replaced with a new connection
	 */
	std::set<int> _available_connections;
	/**
	 * @brief Radius of the RRG around the robot in m
	 */
	double _local_graph_radius;
	/**
	 * @brief Indices of targets (first) in the order of global exploration and the indices of connections
	 * leading to them from the previous target (second)
	 */
	std::vector<std::pair<int, int>> _global_route;
	/**
	 * @brief Index of the entry in the global route that will be the next goal
	 */
	int _next_global_goal;
	/**
	 * @brief While global navigation is active, holds the index of the followed connection and the closest waypoint
	 */
	std::pair<int, int> _active_connections_closest_waypoint;
	/**
	 * @brief Return to the origin node when all nodes targets were explored
	 */
	bool _auto_homing;
	/**
	 * @brief If the navigation to the previous global goal failed this contains the index of the target that failed
	 */
	int _previous_global_goal_failed;

	/**
	 * @brief Returns the second to last node in the given node's connection to the robot in the local graph
	 * as the new connecting node, adds its position to the list of waypoints and the edge's length
	 * between both nodes
	 * @param Index of the node at which to start
	 * @param Reference to the RRG
	 * @param Reference to the list of waypoints where the new connecting node's position will be added
	 * @param Reference to the connecting node for a connection
	 * @param Reference to a length to which the edge length will be added
	 * @return If a connection to an active node of the local graph could be made from this node
	 */
	bool getConnectingNode(int node, rrg_nbv_exploration_msgs::Graph &rrg,
			std::vector<geometry_msgs::Point> &waypoints, int &connecting_node,
			double &length);

	/**
	 * @brief Retrieve the lowest available target index for a new target (uses inactive targets if available)
	 * @return Frontier index for new target
	 */
	int availableFrontierIndex();

	/**
	 * @brief Retrieve the lowest available connection index for a new connection (uses inactive connections if available)
	 * @return Path index for new connection
	 */
	int availablePathIndex();

	/**
	 * @brief Deactivate all pruned targets and remove them from the list of targets if they are at the end
	 * @param Reference to the set of pruned targets
	 */
	void handlePrunedFrontiers(const std::set<int> &pruned_targets);

	/**
	 * @brief Deactivate all pruned connections and remove them from the list of connections if they are at the end
	 * @param Reference to the set of pruned connections
	 */
	void handlePrunedPaths(const std::set<int> &pruned_connections);

	/**
	 * @brief Deactivate the target with the given index by setting it to inactive and removing its connections
	 * @param Index of target to deactivate
	 */
	void deactivateFrontier(int pruned_target);

	/**
	 * @brief Deactivate the connection with the given index by setting it to inactive and removing its waypoints
	 * @param Index of connection to deactivate
	 */
	void deactivatePath(int pruned_connection);

	/**
	 * @brief Add the given target to the global graph by inserting it at an available position marked
	 * by the given index or by adding it to the end of the list if none is available
	 * @param Reference of the target to be inserted
	 */
	void insertFrontierInGg(
			const rrg_nbv_exploration_msgs::GlobalTarget &target);

	/**
	 * @brief Add the given connection to the global graph by inserting it at an available position marked
	 * by the given index or by adding it to the end of the list if none is available
	 * @param Reference of the connection to be inserted
	 */
	void insertPathInGg(
			const rrg_nbv_exploration_msgs::GlobalConnection &connection_between_targets);

	/**
	 * @brief Add the given target index to the list of targets to be pruned from the global graph
	 * including all connections connected to it and remove their connection from the nodes in the RRG
	 * @param Index of the target to be pruned
	 * @param Reference to the set of targets to be pruned
	 * @param Reference to the set of connections to be pruned
	 * @param Reference to the RRG
	 */
	void addFrontierToBePruned(int target_to_prune,
			std::set<int> &pruned_targets, std::set<int> &pruned_connections,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Create a new connection connecting the first and second target at a connecting node
	 *  and add it to the global graph with the waypoints being a union from both connections
	 * @param Index of the first target with a connection to a node in the local graph
	 * @param Index of the second target with a connection to a node in the local graph
	 * @param Index of the connection of the first node
	 * @param Index of the connection of the second node
	 * @param If the connection is made at target one's viewpoint
	 */
	void connectFrontiers(int target_one, int target_two, int connection_one,
			int connection_two, bool connection_at_target_one);

	/**
	 * @brief Check if there are any connections to existing targets at the node or connecting node of a
	 * new target and merge them if there are, the target with the shortest distance to the local
	 * graph remains, all others are deactivated (including their connections)
	 * @param Index of the node in the local graph at which the new target should be placed
	 * @param Reference to the connection of the new target
	 * @param Reference to the RRG
	 * @param Reference to the new target
	 * @return True if the new target was merged into an existing target, false if it can be
	 * placed in the global graph
	 */
	bool tryToMergeAddedFrontiers(int node,
			rrg_nbv_exploration_msgs::GlobalConnection &connection,
			rrg_nbv_exploration_msgs::Graph &rrg,
			rrg_nbv_exploration_msgs::GlobalTarget &target);

	/**
	 * @brief Determine the waypoint closest to the target of the given connection which can be connected to
	 * the new node, the closest waypoint cannot be directly at the target
	 * @param Index of the connection
	 * @param Index of the new node in the RRG
	 * @param List of waypoints of the given connection that could potentially be connected to the new node
	 * ordered ascending by distance to the new node (first=index of the waypoint, second=squared distance)
	 * @param Reference to the RRG
	 * @return Index of a connectable waypoint closest to the target (defaults to last index if no
	 * suitable waypoint was found)
	 */
	int getClosestWaypointToFrontier(int connection, int new_node,
			std::vector<std::pair<int, double>> &waypoints_near_new_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Calculate the resulting connection length if the given connection would be connected to the new node
	 * from the provided closest waypoint
	 * @param Index of the connection
	 * @param Index of the new node in the RRG
	 * @param Index of the closest waypoint to the target of the given connection that can be connected
	 * to the new node
	 * @param Reference to the RRG
	 * @return Resulting connection length when rewiring the connection	 *
	 */
	double calculateNewPathLength(int connection, int new_node,
			int closest_waypoint_to_target,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Rewires the given connection to the provided new node using the given waypoint
	 * @param Index of the connection
	 * @param Index of the closest waypoint to the target of the given connection that can be connected
	 * to the new node
	 * @param New connection length when rewiring the connection
	 * @param Index of the new node in the RRG
	 * @param Reference to the RRG
	 */
	void rewirePathToNewNode(int connection, int closest_waypoint_to_target,
			double new_connection_length, int new_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Attempts to improve existing connections between targets, because the targets from the
	 * given connection was rewired to a new node in the RRG, by checking if a connection via the new node would
	 * reduce the connection length and also creates new connections if a connection to a target is found that did
	 * not exist before
	 * @param Index of the new node in the RRG
	 * @param Index of the connection
	 * @param Reference to the RRG
	 */
	void tryToImproveConnectionsToOtherFrontiers(int new_node, int connection,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Improves an existing connection between two targets because one of the targets
	 * was rewired to a new node in the RRG and the rewiring leads to a shorter connection
	 * @param Index of the connection connecting the targets
	 * @param Index of the connection to the local graph that was rewired
	 * @param Index of the other target which connection will be improved
	 * @param Index of the other target's connection to the local graph
	 */
	void improvePathToConnectedFrontier(int target_connection, int connection,
			int other_target, int other_connection);

	/**
	 * @brief Store the given connection from the provided target to another target as the other target's
	 * connection to local graph (in a reverse order of the provided target has a higher index then the other)
	 * @param Index of the connection connecting both targets
	 * @param Index of the target which will be removed and where the local graph will begin
	 * @param Reference to a list of connection indices which will be connected to the root node of the new
	 * local graph (Indices of the other target's connections to the local graph)
	 */
	void overwritePathToLocalGraph(int connection, int target,
			std::vector<int> &connected_connections);

	/**
	 * @brief Connect all target connections to the local graph directly to the nearest node to the robot
	 * by attaching the connecting node's connection and distance to robot to the target connection
	 * @param Reference to the RRG
	 */
	void connectPathsToLocalGraphToNearestNode(
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Find the shortest connection between active targets in the local graph that do not have a
	 * connecting connection yet
	 * @param List of active targets that must be connected to all other targets
	 * @param Reference to the RRG
	 */
	void establishMissingFrontierToFrontierConnections(
			std::vector<int> active_targets,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @Build a global connection between the given target and the missing target with the provided length
	 * and nodes in the local graph and store it in the global graph
	 * @param Reference to the target for which missing connections are added
	 * @param Index of the other target that currently has no connection to the target
	 * @param Length of the connection through the local graph connecting both targets in m
	 * @param List of nodes in the local graph connecting both targets
	 * @param Reference to the RRG
	 */
	void buildMissingPathBetweenFrontiers(
			rrg_nbv_exploration_msgs::GlobalTarget_<std::allocator<void>> &target,
			int missing_target, double connection_length,
			std::vector<int> &local_connection, rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Iterate over the connections to the node nearest to the robot in the RRG of the connecting nodes
	 * of the target and the missing target and find a mutual node in both connections that leads to a connection
	 * connecting both targets with minimal length
	 * @param Index of the local node connected to the target
	 * @param Index of the local node connected to the target with a missing connection to the former
	 * @param Reference to the max distance threshold that will be updated with the minimal connection length
	 * @param Reference to the RRG
	 */
	void findShortestPathThroughMutualNode(int target_connecting_node,
			int missing_target_connecting_node,
			double &max_distance_threshold,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Get a set of all targets the given target is not yet connected with through a global
	 * connection
	 * @param Reference to the list of active target indices
	 * @param Reference to the target for which missing connections are determined
	 * @return A set of indices of target which do not have a global connection to the given target
	 */
	std::set<int> getMissingFrontierConnections(
			std::vector<int> &active_targets,
			rrg_nbv_exploration_msgs::GlobalTarget &target);

	/**
	 * @brief Calculate the route length by summing up all connection lengths from one target to the next
	 * @param Reference to the route including target indices (first) and indices of the connections to the
	 * next target in the route (second)
	 * @return The route length in m
	 */
	double calculateRouteLength(std::vector<std::pair<int, int>> &route);

	/**
	 * @brief Reverse the order of the targets in the route between and including indices i and k
	 * and update the connections to the next target if necessary
	 * @param Reference to the current route
	 * @param Reference to the new route that this method builds
	 * @param Index of the route index where the reversing begins
	 * @param Index of the route index where the reversing stops
	 * @return If there are connections from every target to the next in the new route
	 */
	bool twoOptSwap(std::vector<std::pair<int, int>> &route,
			std::vector<std::pair<int, int>> &new_route, int i, int k);

	/**
	 * @brief Retrieve the global connection index connecting the current and the next target
	 * @param Current target index
	 * @param Next target index
	 * @return Index of the global connection connecting both targets, -1 if none was found
	 */
	int findPathToNextFrontier(int current_target, int next_target);

	/**
	 * @brief Use the 2-opt method to solve the TSP of visiting every target starting at the local
	 * graph where the robot is
	 * @param A list of active targets which will be regarded for the TSP
	 * @return The indices of the next target (first) and connection to next target (second) or -1 for
	 * both if no solution was found
	 */
	std::pair<int, int> findBestFrontierWithTspTwoOpt(
			std::vector<int> &active_targets);

	/**
	 * @brief Sort targets by the length of the global connections from the robot's positions to
	 * the respective viewpoint
	 * @param First target index
	 * @param Second target index
	 * @return Return true if the connection to target one is shorter than the one to target two
	 */
	bool sortByPathLengthToFrontier(int target_one, int target_two);

	/**
	 * @brief Iterate over all possible 2-opt swaps in the global route and return if one of them
	 * improved the route's connection length
	 * @param Reference to the current route
	 * @return If the route's connection length was improved
	 */
	bool iterateOverTwoOptSwaps(std::vector<std::pair<int, int>> &route);

	/**
	 * @brief Check if the new node is very close to a target, then remove the target from the
	 * global graph or if it is in a connectable range, return the target's position
	 * @param Index of the new node
	 * @param Reference to the set of targets to be pruned
	 * @param Reference to the set of connections to be pruned
	 * @param Reference to the RRG
	 * @return List of positions where to place a new node to prune targets
	 */
	std::vector<geometry_msgs::Point> pruneFrontiersAroundNewNode(int new_node,
			std::set<int> &pruned_targets, std::set<int> &pruned_connections,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Checks if a new node in the RRG can reduce the global connections, if their nearest respective
	 * waypoint would be connected to the new node.
	 * @param Index of the new node
	 * @param Reference to the RRG
	 */
	void prunePathsAroundNewNode(int new_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Checks if targets which have a connection to the target to be pruned can be connected
	 * to the local graph through this target and the new node that replaces it with a shorter length
	 * than their current connection to the local graph and replaces the connection if it does
	 * @param Index of the target to be pruned
	 * @param Index of the new node that will prune the target
	 * @param Distance between the new node and the target to be pruned in m
	 * @param Reference to the RRG
	 */
	void tryToRewirePathsToLocalGraphOverPrunedFrontier(int pruned_target,
			int new_node, double distance,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Check if the two target's with the given connections leading to them fulfill the criteria
	 * to potentially merge them with each other also considering if one of them was already merged
	 * @param Reference to the connection leading to the first target
	 * @param Index of the connection leading to the second target
	 * @param Reference to the first target
	 * @param Reference to the closest mergeable target that was already merged and is unprunable
	 * @param Reference to a list of mergeable targets
	 * @param If the connection length of the connection to the first target is added to the complete length
	 * (optional, defaults to true)
	 */
	void checkFrontierMergeability(
			rrg_nbv_exploration_msgs::GlobalConnection &connection_one, int connection_two,
			rrg_nbv_exploration_msgs::GlobalTarget &target_one,
			MergeableFrontierStruct &closest_mergeable_unprunable_target,
			std::vector<MergeableFrontierStruct> &mergeable_targets,
			double add_connection_one_length = true);

	/**
	 * @brief Removes a target from map and set of connectable targets
	 * @param Frontier index to remove
	 * @param Reference to map with target and connection to local graph indices
	 * @param Reference to set of targets the given target could be connected to
	 */
	void removeFrontierFromConnectableFrontierList(int target,
			std::map<int, int> &connecting_node_targets,
			std::set<int> &new_target_connections);

	/**
	 * @brief Try to merge the given current target and other targets that can be connected at a
	 * newly continued connection, removes merged and therefore pruned targets from the list of possible
	 * new target connections and returns if the current target was merged into another target
	 * @param Index of the current target
	 * @param Index of the current target's connection to the local graph
	 * @param Reference to a map of target indices (first) with their respective connection to the
	 * local graph indices (second)
	 * @param Reference to a set of targets to which the current target has no connection but which
	 * have a connection to the connecting node of this target
	 * @param Reference to the RRG
	 * @return If the current target was merged into another existing target
	 */
	bool tryToMergeContinuedPaths(int current_target, int connection,
			std::map<int, int> &connecting_node_targets,
			std::set<int> &new_target_connections,
			rrg_nbv_exploration_msgs::Graph &rrg);
	void debugRoute(std::vector<std::pair<int, int> > &route, std::string prefix, double length);
};

} /* namespace rrg_nbv_exploration */

#endif /* RRG_NBV_EXPLORATION_SRC_GLOBALGRAPHHANDLER_H_ */
