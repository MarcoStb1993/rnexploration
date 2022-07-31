/*
 * GlobalGraphHandler.cpp
 *
 *  Created on: Mar 4, 2022
 *      Author: marco
 */

#include <rrg_nbv_exploration/GlobalGraphHandler.h>

namespace rrg_nbv_exploration {

GlobalGraphHandler::GlobalGraphHandler() {
	ros::NodeHandle private_nh("~");
	private_nh.param("local_graph_radius", _local_graph_radius, 5.0);
	private_nh.param("inflation_active", _inflation_active, true);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("robot_width", _robot_width, 1.0);
	_half_path_box_distance_thres = sqrt(
			pow(_robot_radius, 2) - pow(_robot_width / 2, 2));
	double max_edge_distance;
	double min_edge_distance;
	private_nh.param("max_edge_distance", max_edge_distance, 1.0);
	private_nh.param("min_edge_distance", min_edge_distance, 1.0);
	_max_edge_distance_squared = pow(max_edge_distance, 2);
	_min_edge_distance_squared = pow(min_edge_distance, 2);
	_robot_radius_squared = pow(_robot_radius, 2);
	private_nh.param("auto_homing", _auto_homing, false);
	ros::NodeHandle nh("rne");
	_global_graph_publisher =
			nh.advertise<rrg_nbv_exploration_msgs::GlobalGraph>("globalgraph",
					1);

	_local_graph_radius = std::max(_local_graph_radius, 2 * _robot_radius); // cannot fall below the robot diameter

	_global_graph_searcher.reset(new GlobalGraphSearcher());
	_global_path_waypoint_searcher.reset(new GlobalPathWaypointSearcher());
}

GlobalGraphHandler::~GlobalGraphHandler() {
}

void GlobalGraphHandler::initialize(rrg_nbv_exploration_msgs::Node &root,
		std::shared_ptr<GraphPathCalculator> graph_path_calculator,
		std::shared_ptr<GraphSearcher> graph_searcher,
		std::shared_ptr<CollisionChecker> collision_checker) {
	_gg.frontiers.clear();
	_gg.paths.clear();
	_graph_path_calculator = std::move(graph_path_calculator);
	_graph_searcher = std::move(graph_searcher);
	_collision_checker = std::move(collision_checker);
	_gg.header.frame_id = "map";
	_gg.ns = "globalgraph";
	_gg.frontiers_counter = 0;
	_gg.paths_counter = 0;
	rrg_nbv_exploration_msgs::GlobalFrontier frontier;
	frontier.index = _gg.frontiers_counter++;
	frontier.viewpoint = root.position;
	rrg_nbv_exploration_msgs::GlobalPath path;
	path.index = _gg.paths_counter++;
	path.frontier = rrg_nbv_exploration_msgs::GlobalPath::ORIGIN;
	path.connection = rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH;
	path.connecting_node = root.index;
	path.waypoints.push_back(root.position);
	path.length = 0;
	frontier.paths.push_back(path.index);
	frontier.paths_counter++;
	_gg.frontiers.push_back(frontier);
	_gg.paths.push_back(path);
	root.connected_to.push_back(path.index);
	_available_frontiers.clear();
	_available_paths.clear();
	_global_route.clear();
	_next_global_goal = -1;
	_previous_global_goal_failed = false;
	_active_paths_closest_waypoint = std::make_pair(-1, -1);
	_global_graph_searcher->initialize(_gg);
}

void GlobalGraphHandler::publishGlobalGraph() {
	_global_graph_publisher.publish(_gg);
}

bool GlobalGraphHandler::getConnectingNode(int node,
		rrg_nbv_exploration_msgs::Graph &rrg,
		std::vector<geometry_msgs::Point> &waypoints, int &connecting_node,
		double &length) {
	if (rrg.nodes.at(node).path_to_robot.size() > 1) {
		int next_node = rrg.nodes.at(node).path_to_robot.size() - 2;
		int edge = _graph_path_calculator->findExistingEdge(rrg,
				rrg.nodes.at(node).path_to_robot.at(next_node + 1),
				rrg.nodes.at(node).path_to_robot.at(next_node));
		if (edge >= rrg.edges.size()) {
			return false;
		}
		if (edge != -1) {
			connecting_node = rrg.nodes.at(node).path_to_robot.at(next_node);
			length += rrg.edges.at(edge).length;
			waypoints.push_back(
					rrg.nodes.at(rrg.nodes.at(node).path_to_robot.at(next_node)).position);
			return true;
		}
	}
	return false;
}

void GlobalGraphHandler::insertFrontierInGg(
		const rrg_nbv_exploration_msgs::GlobalFrontier &frontier) {
	if (!_available_frontiers.empty()) {
		_gg.frontiers.at(*_available_frontiers.begin()) = frontier;
		_available_frontiers.erase(_available_frontiers.begin());
	} else {
		_gg.frontiers.push_back(frontier);
		_gg.frontiers_counter++;
	}
}

void GlobalGraphHandler::insertPathInGg(
		const rrg_nbv_exploration_msgs::GlobalPath &path_between_frontiers) {
	if (!_available_paths.empty()) {
		_gg.paths.at(*_available_paths.begin()) = path_between_frontiers;
		_available_paths.erase(_available_paths.begin());
	} else {
		_gg.paths.push_back(path_between_frontiers);
		_gg.paths_counter++;
	}
}

void GlobalGraphHandler::addFrontier(int node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	rrg_nbv_exploration_msgs::GlobalFrontier frontier;
	rrg_nbv_exploration_msgs::GlobalPath path;
	frontier.index = availableFrontierIndex();
	frontier.viewpoint = rrg.nodes.at(node).position;
	frontier.merged_distance = 0.0;
	path.connection = rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH;
	path.waypoints.push_back(rrg.nodes.at(node).position);
	if (!getConnectingNode(node, rrg, path.waypoints, path.connecting_node,
			path.length)) {
		ROS_WARN_STREAM(
				"Unable to add frontier for node " << node << ", found no connection to the RRG");
		return;
	}
	if (tryToMergeAddedFrontiers(node, path, rrg, frontier)) { // if new frontier is merged into existing
		return;
	}
	path.index = availablePathIndex();
	path.frontier = frontier.index;
	frontier.paths.push_back(path.index);
	frontier.paths_counter++;
	insertFrontierInGg(frontier);
	insertPathInGg(path);
	for (auto connected_path : rrg.nodes.at(node).connected_to) { // new frontier is in path of existing frontiers
		connectFrontiers(frontier.index, _gg.paths.at(connected_path).frontier,
				path.index, connected_path, true);
	}
	for (auto connected_path : rrg.nodes.at(path.connecting_node).connected_to) { // connecting node is in path of existing frontiers, connect frontiers
		connectFrontiers(frontier.index, _gg.paths.at(connected_path).frontier,
				path.index, connected_path, false);
	}
	rrg.nodes.at(path.connecting_node).connected_to.push_back(path.index);
	_global_graph_searcher->rebuildIndex(_gg);
}

void GlobalGraphHandler::continuePath(int node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<geometry_msgs::Point> additional_waypoints;
	int connecting_node;
	double length = 0;
	if (!getConnectingNode(node, rrg, additional_waypoints, connecting_node,
			length)) {
		ROS_WARN_STREAM(
				"Unable to continue paths at node " << node << ", found no connection to the RRG");
		return;
	}
	std::map<int, int> connecting_node_frontiers; // frontier index (first), path to connected node index (second)
	for (auto connected_path : rrg.nodes.at(connecting_node).connected_to) { // get list of already present path connections at new connected node in local graph
		connecting_node_frontiers.insert(
				{ _gg.paths.at(connected_path).frontier, connected_path });
	}
	std::vector<int> connected_paths = rrg.nodes.at(node).connected_to; //create copy because connected paths can be pruned in the for loop
	for (int path : connected_paths) {
		if (path < _gg.paths_counter && !_gg.paths.at(path).inactive
				&& _gg.paths.at(path).connecting_node == node) { //check if the path was not pruned (and replaced)
			_gg.paths.at(path).waypoints.insert(
					_gg.paths.at(path).waypoints.end(),
					additional_waypoints.begin(), additional_waypoints.end()); // add additional waypoints to path
			_gg.paths.at(path).connecting_node = connecting_node;
			_gg.paths.at(path).length += length;
			rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.push_back(
					path);
			if (connecting_node_frontiers.size() > 0) {
				int current_frontier = _gg.paths.at(path).frontier; // look for new connections with this frontier
				std::set<int> new_frontier_connections;	// set of existing connected frontiers to the connecting node which will be reduced by the frontiers already connected to the current frontier
				for (auto elem : connecting_node_frontiers) {
					new_frontier_connections.insert(elem.first);
				}
				for (auto frontier_path : _gg.frontiers.at(current_frontier).paths) {
					if (_gg.paths.at(frontier_path).connection
							!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) { // connected to another frontier
						new_frontier_connections.erase(
								current_frontier // current frontier can be frontier or connection at connecting path
								== _gg.paths.at(frontier_path).frontier ?
										_gg.paths.at(frontier_path).connection :
										_gg.paths.at(frontier_path).frontier); // remove from new frontiers (if present)
					}
				}
				if (current_frontier
						!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
						&& tryToMergeContinuedPaths(current_frontier, path,
								connecting_node_frontiers,
								new_frontier_connections, rrg)) //if current frontier was pruned
					continue;
				for (auto new_frontier_connection : new_frontier_connections) { //remaining missing frontier connections
					connectFrontiers(current_frontier, new_frontier_connection,
							path,
							connecting_node_frontiers.at(
									new_frontier_connection), false);
				}
			}
		}
	}
}

bool GlobalGraphHandler::tryToMergeContinuedPaths(int current_frontier,
		int path, std::map<int, int> &connecting_node_frontiers,
		std::set<int> &new_frontier_connections,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	bool pruned_current_frontier = false;
	std::vector<MergeableFrontierStruct> mergeable_frontiers;
	MergeableFrontierStruct closest_mergeable_unprunable_frontier(-1,
			std::numeric_limits<double>::infinity(), 0);
	for (auto new_frontier_connection : new_frontier_connections) { //check if can be merged
		checkFrontierMergeability(_gg.paths.at(path),
				connecting_node_frontiers.at(new_frontier_connection),
				_gg.frontiers.at(current_frontier),
				closest_mergeable_unprunable_frontier, mergeable_frontiers);
	}
	std::set<int> pruned_frontiers;
	std::set<int> pruned_paths;
	if (closest_mergeable_unprunable_frontier.frontier != -1) { //merge this frontier into closest mergeable unprunable frontier
		_gg.frontiers.at(closest_mergeable_unprunable_frontier.frontier).merged_distance =
				std::max(
						_gg.frontiers.at(current_frontier).merged_distance
								+ closest_mergeable_unprunable_frontier.distance_between_frontiers,
						_gg.frontiers.at(
								closest_mergeable_unprunable_frontier.frontier).merged_distance);
		addFrontierToBePruned(current_frontier, pruned_frontiers, pruned_paths,
				rrg);
		pruned_current_frontier = true;
	} else if (mergeable_frontiers.size()) {
		int remaining_frontier = current_frontier;
		double shortest_path_length = _gg.paths.at(path).length;
		std::set<std::pair<double, int>> merged_distances_to_frontiers; //merged distance to frontier (first) and frontier index (second) ordered by merged distance
		merged_distances_to_frontiers.insert(
				std::make_pair(
						_gg.frontiers.at(current_frontier).merged_distance,
						current_frontier));
		double distance_current_frontier_to_remaining_frontier = 0;
		for (auto mergeable_frontier : mergeable_frontiers) { // find remaining frontier
			merged_distances_to_frontiers.insert(
					std::make_pair(
							_gg.frontiers.at(mergeable_frontier.frontier).merged_distance
									+ mergeable_frontier.distance_between_frontiers,
							mergeable_frontier.frontier));
			if (mergeable_frontier.path_length_to_local_graph
					<= shortest_path_length) {
				if (remaining_frontier != current_frontier) { // not in graph, cannot be pruned
					addFrontierToBePruned(remaining_frontier, pruned_frontiers,
							pruned_paths, rrg);
					removeFrontierFromConnectableFrontierList(
							remaining_frontier, connecting_node_frontiers,
							new_frontier_connections);
				}
				remaining_frontier = mergeable_frontier.frontier;
				shortest_path_length =
						mergeable_frontier.path_length_to_local_graph;
				distance_current_frontier_to_remaining_frontier =
						mergeable_frontier.distance_between_frontiers;
			} else {
				addFrontierToBePruned(mergeable_frontier.frontier,
						pruned_frontiers, pruned_paths, rrg);
				removeFrontierFromConnectableFrontierList(
						mergeable_frontier.frontier, connecting_node_frontiers,
						new_frontier_connections);
			}
		}
		if (remaining_frontier != current_frontier) { // don't add frontier, it was merged into existing
			double max_relevant_merged_distance = 0;
			if (merged_distances_to_frontiers.rbegin()->second
					!= remaining_frontier) { //check if max merged distance is not merged distance to remaining frontier, because this would add distance_current_frontier_to_remaining_frontier twice
				max_relevant_merged_distance =
						merged_distances_to_frontiers.rbegin()->first
								+ distance_current_frontier_to_remaining_frontier;
			} else { //otherwise take second merged distance which is to another frontier
				max_relevant_merged_distance = std::next(
						merged_distances_to_frontiers.rbegin())->first
						+ distance_current_frontier_to_remaining_frontier;
			}
			if (max_relevant_merged_distance > _local_graph_radius) { //if new merged distance violates the constraint, prune the remaining frontier and keep the current frontier
				addFrontierToBePruned(remaining_frontier, pruned_frontiers,
						pruned_paths, rrg);
				removeFrontierFromConnectableFrontierList(remaining_frontier,
						connecting_node_frontiers, new_frontier_connections);
				_gg.frontiers.at(current_frontier).merged_distance =
						merged_distances_to_frontiers.rbegin()->first;
			} else { //merge like all other frontiers were merged into current frontier and then into remaining frontier
				addFrontierToBePruned(current_frontier, pruned_frontiers,
						pruned_paths, rrg);
				_gg.frontiers.at(remaining_frontier).merged_distance = std::max(
						max_relevant_merged_distance,
						_gg.frontiers.at(remaining_frontier).merged_distance);
				pruned_current_frontier = true;
			}
		} else {
			_gg.frontiers.at(current_frontier).merged_distance =
					merged_distances_to_frontiers.rbegin()->first;
		}
	}
	if (pruned_frontiers.size() > 0) {
		handlePrunedFrontiers(pruned_frontiers);
		handlePrunedPaths(pruned_paths);
	}
	return pruned_current_frontier;
}

void GlobalGraphHandler::removeFrontierFromConnectableFrontierList(int frontier,
		std::map<int, int> &connecting_node_frontiers,
		std::set<int> &new_frontier_connections) {
	auto it = connecting_node_frontiers.find(frontier);
	if (it != connecting_node_frontiers.end()) {
		connecting_node_frontiers.erase(it);
	}
	new_frontier_connections.erase(frontier);
}

bool GlobalGraphHandler::tryToMergeAddedFrontiers(int node,
		rrg_nbv_exploration_msgs::GlobalPath &path,
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::GlobalFrontier &frontier) {
	bool pruned_added_frontier = false;
	if (!rrg.nodes.at(node).connected_to.empty()
			|| !rrg.nodes.at(path.connecting_node).connected_to.empty()) {
		std::vector<MergeableFrontierStruct> mergeable_frontiers;
		MergeableFrontierStruct closest_mergeable_unprunable_frontier(-1,
				std::numeric_limits<double>::infinity(), 0);
		for (auto connected_path : rrg.nodes.at(node).connected_to) {
			checkFrontierMergeability(path, connected_path, frontier,
					closest_mergeable_unprunable_frontier, mergeable_frontiers,
					false);
		}
		for (auto connected_path : rrg.nodes.at(path.connecting_node).connected_to) {
			checkFrontierMergeability(path, connected_path, frontier,
					closest_mergeable_unprunable_frontier, mergeable_frontiers);
		}
		if (closest_mergeable_unprunable_frontier.frontier != -1) { //merge this frontier into closest mergeable unprunable frontier
			_gg.frontiers.at(closest_mergeable_unprunable_frontier.frontier).merged_distance =
					std::max(
							frontier.merged_distance
									+ closest_mergeable_unprunable_frontier.distance_between_frontiers,
							_gg.frontiers.at(
									closest_mergeable_unprunable_frontier.frontier).merged_distance);
			pruned_added_frontier = true;
		} else if (mergeable_frontiers.size()) {
			std::set<int> pruned_frontiers;
			std::set<int> pruned_paths;
			int remaining_frontier = frontier.index;
			double shortest_path_length = path.length;
			std::set<std::pair<double, int>> merged_distances_to_frontiers; //merged distance to frontier (first) and frontier index (second) ordered by merged distance
			merged_distances_to_frontiers.insert(
					std::make_pair(frontier.merged_distance, frontier.index));
			double distance_current_frontier_to_remaining_frontier = 0;
			for (auto mergeable_frontier : mergeable_frontiers) { // find remaining frontier
				merged_distances_to_frontiers.insert(
						std::make_pair(
								_gg.frontiers.at(mergeable_frontier.frontier).merged_distance
										+ mergeable_frontier.distance_between_frontiers,
								mergeable_frontier.frontier));
				if (mergeable_frontier.path_length_to_local_graph
						<= shortest_path_length) {
					if (remaining_frontier != frontier.index) { // not in graph, cannot be pruned
						addFrontierToBePruned(remaining_frontier,
								pruned_frontiers, pruned_paths, rrg);
					}
					remaining_frontier = mergeable_frontier.frontier;
					shortest_path_length =
							mergeable_frontier.path_length_to_local_graph;
					distance_current_frontier_to_remaining_frontier =
							mergeable_frontier.distance_between_frontiers;
				} else {
					addFrontierToBePruned(mergeable_frontier.frontier,
							pruned_frontiers, pruned_paths, rrg);
				}
			}
			if (remaining_frontier != frontier.index) { // don't add frontier, it was merged into existing
				double max_relevant_merged_distance = 0;
				if (merged_distances_to_frontiers.rbegin()->second
						== remaining_frontier) { //check if max merged distance is not merged distance to remaining frontier, because this would add distance_current_frontier_to_remaining_frontier twice
					max_relevant_merged_distance =
							merged_distances_to_frontiers.rbegin()->first
									+ distance_current_frontier_to_remaining_frontier;
				} else { //otherwise take second merged distance which is to another frontier
					max_relevant_merged_distance = std::next(
							merged_distances_to_frontiers.rbegin())->first
							+ distance_current_frontier_to_remaining_frontier;
				}
				if (max_relevant_merged_distance > _local_graph_radius) { //if new merged distance violates the constraint, prune the remaining frontier and keep the added frontier
					addFrontierToBePruned(remaining_frontier, pruned_frontiers,
							pruned_paths, rrg);
					frontier.merged_distance =
							merged_distances_to_frontiers.rbegin()->first;
				} else { //merge like all other frontiers were merged into current frontier and then into remaining frontier
					_gg.frontiers.at(remaining_frontier).merged_distance =
							std::max(max_relevant_merged_distance,
									_gg.frontiers.at(remaining_frontier).merged_distance);
					pruned_added_frontier = true;
				}
			} else {
				frontier.merged_distance =
						merged_distances_to_frontiers.rbegin()->first;
			}
			if (pruned_frontiers.size() > 0) {
				handlePrunedFrontiers(pruned_frontiers);
				handlePrunedPaths(pruned_paths);
			}
			if (!pruned_added_frontier) {
				frontier.index = availableFrontierIndex();
			}
		}
	}
	return pruned_added_frontier;
}

void GlobalGraphHandler::checkFrontierMergeability(
		rrg_nbv_exploration_msgs::GlobalPath &path_one, int path_two,
		rrg_nbv_exploration_msgs::GlobalFrontier &frontier_one,
		MergeableFrontierStruct &closest_mergeable_unprunable_frontier,
		std::vector<MergeableFrontierStruct> &mergeable_frontiers,
		double add_path_one_length) {
	double combined_length = _gg.paths.at(path_two).length
			+ (add_path_one_length ? path_one.length : 0);
	double distance = _graph_path_calculator->distance2d(frontier_one.viewpoint,
			_gg.frontiers.at(_gg.paths.at(path_two).frontier).viewpoint);
	if (_gg.paths.at(path_two).frontier
			!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
			&& combined_length <= 2 * _local_graph_radius
			&& distance <= _local_graph_radius) {
		double frontier_one_new_merged_distance = frontier_one.merged_distance
				+ distance;
		double frontier_two_new_merged_distance = _gg.frontiers.at(
				_gg.paths.at(path_two).frontier).merged_distance + distance;
		if (frontier_one_new_merged_distance <= _local_graph_radius
				|| frontier_two_new_merged_distance <= _local_graph_radius) {
			if (frontier_two_new_merged_distance > _local_graph_radius) { //frontier one won't have the frontiers merged into frontier two in its local graph radius, therefore frontier two is unpruneable
				if (combined_length
						< closest_mergeable_unprunable_frontier.path_length_to_local_graph) {
					closest_mergeable_unprunable_frontier =
							MergeableFrontierStruct(
									_gg.paths.at(path_two).frontier,
									_gg.paths.at(path_two).length, distance);
				}
			} else {
				mergeable_frontiers.emplace_back(
						_gg.paths.at(path_two).frontier,
						_gg.paths.at(path_two).length, distance);
			}
		}
	}
}

void GlobalGraphHandler::connectFrontiers(int frontier_one, int frontier_two,
		int path_one, int path_two, bool connection_at_frontier) {
	int frontier;
	int connection;
	int frontier_path;
	int connection_path;
	bool skip_frontier_path = false;
	bool skip_connection_path = false;
	if (frontier_one > frontier_two) { // higher index frontier is always frontier, the other one connection, path always starts at frontier
		frontier = frontier_one;
		connection = frontier_two;
		frontier_path = path_one;
		connection_path = path_two;
		if (connection_at_frontier)
			skip_frontier_path = true;
	} else {
		frontier = frontier_two;
		connection = frontier_one;
		frontier_path = path_two;
		connection_path = path_one;
		if (connection_at_frontier)
			skip_connection_path = true;
	}
	rrg_nbv_exploration_msgs::GlobalPath path_between_frontiers;
	path_between_frontiers.index = availablePathIndex();
	path_between_frontiers.frontier = frontier;
	path_between_frontiers.connection = connection;
	path_between_frontiers.length = 0;
	if (!skip_frontier_path) {
		path_between_frontiers.waypoints.insert(
				path_between_frontiers.waypoints.begin(),
				_gg.paths.at(frontier_path).waypoints.begin(),
				_gg.paths.at(frontier_path).waypoints.end() - 1); // omit last element to avoid duplicate waypoint
		path_between_frontiers.length += _gg.paths.at(frontier_path).length;
	}
	if (!skip_connection_path) {
		path_between_frontiers.waypoints.insert(
				path_between_frontiers.waypoints.end(),
				_gg.paths.at(connection_path).waypoints.rbegin(),
				_gg.paths.at(connection_path).waypoints.rend());
		path_between_frontiers.length += _gg.paths.at(connection_path).length;
	}
	path_between_frontiers.connecting_node = -1;
	_gg.frontiers.at(frontier).paths.push_back(path_between_frontiers.index);
	_gg.frontiers.at(frontier).paths_counter++;
	_gg.frontiers.at(connection).paths.push_back(path_between_frontiers.index);
	_gg.frontiers.at(connection).paths_counter++;
	insertPathInGg(path_between_frontiers);
}

int GlobalGraphHandler::availableFrontierIndex() {
	if (!_available_frontiers.empty()) {
		return *_available_frontiers.begin();
	} else {
		return _gg.frontiers_counter;
	}
}

int GlobalGraphHandler::availablePathIndex() {
	if (!_available_paths.empty()) {
		return *_available_paths.begin();
	} else {
		return _gg.paths_counter;
	}
}

void GlobalGraphHandler::addFrontierToBePruned(int frontier_to_prune,
		std::set<int> &pruned_frontiers, std::set<int> &pruned_paths,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	pruned_frontiers.insert(frontier_to_prune);
	for (auto path : _gg.frontiers.at(frontier_to_prune).paths) {
		pruned_paths.insert(path);
		if (_gg.paths.at(path).connecting_node != -1) // remove possible connection from node in RRG
			rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.erase(
					std::remove(
							rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.begin(),
							rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.end(),
							path),
					rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.end());
	}
}

void GlobalGraphHandler::handlePrunedFrontiers(
		const std::set<int> &pruned_frontiers) {
	bool removed_frontiers = false;
	auto pruned_frontier = pruned_frontiers.rbegin();
	while (pruned_frontier != pruned_frontiers.rend()) { // remove or add inactive frontiers
		deactivateFrontier(*pruned_frontier);
		if (*pruned_frontier == _gg.frontiers_counter - 1) {
			int next_pruned_frontier =
					std::next(pruned_frontier) == pruned_frontiers.rend() ?
							0 : *std::next(pruned_frontier); // if last pruned frontier, try to remove inactive frontiers up to index 1 (origin can't be pruned)
			for (int i = *pruned_frontier; i > next_pruned_frontier; i--) {
				if (_gg.frontiers.at(i).inactive) {
					_gg.frontiers.pop_back();
					_gg.frontiers_counter--;
					removed_frontiers = true;
				} else {
					break;
				}
			}
		} else {
			_available_frontiers.insert(*pruned_frontier);
		}
		pruned_frontier++;
	}
	_global_graph_searcher->rebuildIndex(_gg);
	if (removed_frontiers) {
		int removals = 0;
		for (auto it = _available_frontiers.begin();
				it != _available_frontiers.end();) {
			if (*it >= _gg.frontiers_counter) {
				it = _available_frontiers.erase(it);
				removals++;
			} else {
				++it;
			}
		}
	}
}

void GlobalGraphHandler::handlePrunedPaths(const std::set<int> &pruned_paths) {
	bool removed_paths = false;
	auto pruned_path = pruned_paths.rbegin();
	while (pruned_path != pruned_paths.rend()) { // remove or add inactive paths
		deactivatePath(*pruned_path);
		if (*pruned_path == _gg.paths_counter - 1) {
			int next_pruned_path =
					std::next(pruned_path) == pruned_paths.rend() ?
							-1 : *std::next(pruned_path); // if last pruned path, try to remove inactive paths up to index 0
			for (int i = *pruned_path; i > next_pruned_path; i--) {
				if (_gg.paths.at(i).inactive) {
					_gg.paths.pop_back();
					_gg.paths_counter--;
					removed_paths = true;
				} else {
					break;
				}
			}
		} else {
			_available_paths.insert(*pruned_path);
		}
		pruned_path++;
	}
	if (removed_paths) {
		int removals = 0;
		for (auto it = _available_paths.begin(); it != _available_paths.end();
				) {
			if (*it >= _gg.paths_counter) {
				it = _available_paths.erase(it);
				removals++;
			} else {
				++it;
			}
		}
	}
}

void GlobalGraphHandler::deactivateFrontier(int pruned_frontier) {
	_gg.frontiers.at(pruned_frontier).inactive = true;
	_gg.frontiers.at(pruned_frontier).merged_distance = 0.0;
	_gg.frontiers.at(pruned_frontier).paths.clear();
	_gg.frontiers.at(pruned_frontier).paths_counter = 0;
	_gg.frontiers.at(pruned_frontier).viewpoint.x = 0;
	_gg.frontiers.at(pruned_frontier).viewpoint.y = 0;
}

void GlobalGraphHandler::deactivatePath(int pruned_path) {
	if (_gg.paths.at(pruned_path).frontier < _gg.frontiers_counter
			&& !_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).inactive) { // remove path from list at frontier
		_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.erase(
				std::remove(
						_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.begin(),
						_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.end(),
						pruned_path),
				_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.end());
		_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths_counter--;
	}
	if (_gg.paths.at(pruned_path).connection
			> rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH
			&& _gg.paths.at(pruned_path).connection < _gg.frontiers_counter
			&& !_gg.frontiers.at(_gg.paths.at(pruned_path).connection).inactive) { // remove path from list at connected frontier
		_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.erase(
				std::remove(
						_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.begin(),
						_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.end(),
						pruned_path),
				_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.end());
		_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths_counter--;
	}
	_gg.paths.at(pruned_path).inactive = true;
	_gg.paths.at(pruned_path).waypoints.clear();
	_gg.paths.at(pruned_path).length = 0;
}

std::vector<geometry_msgs::Point> GlobalGraphHandler::pruneFrontiersAndPathsAroundNewNode(
		rrg_nbv_exploration_msgs::Graph &rrg, int new_node) {
	std::set<int> pruned_frontiers;
	std::set<int> pruned_paths;
	std::vector<geometry_msgs::Point> new_node_positions =
			pruneFrontiersAroundNewNode(new_node, pruned_frontiers,
					pruned_paths, rrg);
	if (pruned_frontiers.size() > 0) {
		handlePrunedFrontiers(pruned_frontiers);
		handlePrunedPaths(pruned_paths);
	}
	prunePathsAroundNewNode(new_node, rrg);
	return new_node_positions;
}

std::vector<geometry_msgs::Point> GlobalGraphHandler::pruneFrontiersAroundNewNode(
		int new_node, std::set<int> &pruned_frontiers,
		std::set<int> &pruned_paths, rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<geometry_msgs::Point> new_node_positions;
	std::vector<std::pair<int, double>> frontiers_near_new_node =
			_global_graph_searcher->searchInRadius(
					rrg.nodes.at(new_node).position,
					_inflation_active ?
							pow(
									sqrt(
											rrg.nodes.at(new_node).squared_radius
													- pow(_robot_width / 2, 2))
											+ _half_path_box_distance_thres,
									2) :
							_max_edge_distance_squared); // frontiers in connectable distance
	for (auto frontier_near_new_node : frontiers_near_new_node) {
		if (frontier_near_new_node.first
				!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN) {
			if (frontier_near_new_node.second
					<= (_inflation_active ?
							_robot_radius_squared : _min_edge_distance_squared)) // new node nearly at frontier position, prune it
					{
				tryToRewirePathsToLocalGraphOverPrunedFrontier(
						frontier_near_new_node.first, new_node,
						sqrt(frontier_near_new_node.second), rrg);
				addFrontierToBePruned(frontier_near_new_node.first,
						pruned_frontiers, pruned_paths, rrg);
			} else // try to place a node at frontier position to prune it
			{
				new_node_positions.push_back(
						_gg.frontiers.at(frontier_near_new_node.first).viewpoint);
			}
		}
	}
	return new_node_positions;
}

void GlobalGraphHandler::tryToRewirePathsToLocalGraphOverPrunedFrontier(
		int pruned_frontier, int new_node, double distance,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto path : _gg.frontiers.at(pruned_frontier).paths) { //iterate over all frontier to frontier paths of the frontier to be pruned
		if (_gg.paths.at(path).connection
				!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) { //ignore path to local graph
			int other_frontier =
					pruned_frontier == _gg.paths.at(path).frontier ?
							_gg.paths.at(path).connection :
							_gg.paths.at(path).frontier;
			int other_frontiers_path_to_local_graph = _gg.frontiers.at(
					other_frontier).paths.front();
			double new_distance = distance + _gg.paths.at(path).length;
			if (new_distance
					< _gg.paths.at(other_frontiers_path_to_local_graph).length) // new path length through pruned frontier shorter than current
					{
				_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.clear();
				if (pruned_frontier == _gg.paths.at(path).frontier) // reverse waypoints
						{
					_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.insert(
							_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.begin(),
							_gg.paths.at(path).waypoints.rbegin(),
							_gg.paths.at(path).waypoints.rend());
				} else {
					_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.insert(
							_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.begin(),
							_gg.paths.at(path).waypoints.begin(),
							_gg.paths.at(path).waypoints.end());
				}
				_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.push_back(
						rrg.nodes.at(new_node).position);
				_gg.paths.at(other_frontiers_path_to_local_graph).length =
						new_distance;
				int connecting_node = _gg.paths.at(
						other_frontiers_path_to_local_graph).connecting_node;
				rrg.nodes.at(connecting_node).connected_to.erase(
						std::remove(
								rrg.nodes.at(connecting_node).connected_to.begin(),
								rrg.nodes.at(connecting_node).connected_to.end(),
								other_frontiers_path_to_local_graph),
						rrg.nodes.at(connecting_node).connected_to.end());
				_gg.paths.at(other_frontiers_path_to_local_graph).connecting_node =
						new_node;
				rrg.nodes.at(new_node).connected_to.push_back(
						other_frontiers_path_to_local_graph);
			}
		}
	}
}

void GlobalGraphHandler::prunePathsAroundNewNode(int new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto frontier : _gg.frontiers) { // iterate over all paths to local graph
		if (frontier.inactive) {
			continue;
		}
		int path = frontier.paths.front(); // path to local graph
		_global_path_waypoint_searcher->rebuildIndex(_gg.paths.at(path));
		std::vector<std::pair<int, double>> waypoints_near_new_node =
				_global_path_waypoint_searcher->searchInRadius(
						rrg.nodes.at(new_node).position,
						_inflation_active ?
								pow(
										_robot_radius
												+ rrg.nodes.at(new_node).radius,
										2) :
								_max_edge_distance_squared); // all waypoints that can be connected to the new node
		if (waypoints_near_new_node.size() > 0) {

			int closest_waypoint_to_frontier = getClosestWaypointToFrontier(
					path, new_node, waypoints_near_new_node, rrg);
			if (closest_waypoint_to_frontier
					< (_gg.paths.at(path).waypoints.size() - 1)) { // if path can be pruned
				double new_path_length = calculateNewPathLength(path, new_node,
						closest_waypoint_to_frontier, rrg);
				if (new_path_length < _gg.paths.at(path).length) { // only re-route path if length decreases
					rewirePathToNewNode(path, closest_waypoint_to_frontier,
							new_path_length, new_node, rrg);
					tryToImproveConnectionsToOtherFrontiers(new_node, path,
							rrg);
					rrg.nodes.at(new_node).connected_to.push_back(path); // add after improving connections to not filter it out during improving
				}
			}
		}
	}
}

int GlobalGraphHandler::getClosestWaypointToFrontier(int path, int new_node,
		std::vector<std::pair<int, double>> &waypoints_near_new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	int closest_waypoint_to_frontier = _gg.paths.at(path).waypoints.size() - 1; // index of last waypoint in list
// find connectable waypoint closest to the frontier (most path reduction)
	if (_inflation_active) {
		for (auto waypoint_near_new_node : waypoints_near_new_node) {
			if (waypoint_near_new_node.first < closest_waypoint_to_frontier) { // new node must overlap with waypoint at frontier for re-wiring
				closest_waypoint_to_frontier = waypoint_near_new_node.first;
			}
		}
	} else {
		std::sort(waypoints_near_new_node.begin(),
				waypoints_near_new_node.end(),
				[](std::pair<int, double> first_waypoint,
						std::pair<int, double> second_waypoint) {
					return first_waypoint.first < second_waypoint.first;
				}); // sort waypoints ascending by index
		for (auto waypoint_near_new_node : waypoints_near_new_node) {
			if (_collision_checker->checkConnectionToFrontierPathWaypoint(rrg,
					new_node,
					_gg.paths.at(path).waypoints.at(
							waypoint_near_new_node.first),
					sqrt(waypoint_near_new_node.second))) { // check if a connection can be made
				closest_waypoint_to_frontier = waypoint_near_new_node.first;
				break;
			}
		}
	}
	return closest_waypoint_to_frontier;
}

double GlobalGraphHandler::calculateNewPathLength(int path, int new_node,
		int closest_waypoint_to_frontier,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	double new_length = 0;
	double distance_to_new_node = sqrt(
			pow(
					_gg.paths.at(path).waypoints.at(
							closest_waypoint_to_frontier).x
							- rrg.nodes.at(new_node).position.x, 2)
					+ pow(
							_gg.paths.at(path).waypoints.at(
									closest_waypoint_to_frontier).y
									- rrg.nodes.at(new_node).position.y, 2)); // distance between closest waypoint and new connecting node
	if ((_gg.paths.at(path).waypoints.size() - 1 - closest_waypoint_to_frontier)
			> closest_waypoint_to_frontier) { //less waypoints remaining than pruned
		for (int i = 1; i <= closest_waypoint_to_frontier; i++) { // calculate distances between waypoints to remain in path
			new_length += sqrt(
					pow(
							_gg.paths.at(path).waypoints.at(i).x
									- _gg.paths.at(path).waypoints.at(i - 1).x,
							2)
							+ pow(
									_gg.paths.at(path).waypoints.at(i).y
											- _gg.paths.at(path).waypoints.at(
													i - 1).y, 2));
		}
	} else { //less waypoints pruned than remaining
		new_length = _gg.paths.at(path).length;
		for (int i = _gg.paths.at(path).waypoints.size() - 1;
				i > closest_waypoint_to_frontier; i--) { // calculate distances between waypoints to be pruned from path
			new_length -= sqrt(
					pow(
							_gg.paths.at(path).waypoints.at(i).x
									- _gg.paths.at(path).waypoints.at(i - 1).x,
							2)
							+ pow(
									_gg.paths.at(path).waypoints.at(i).y
											- _gg.paths.at(path).waypoints.at(
													i - 1).y, 2));
		}
	}
	return (new_length + distance_to_new_node);
}

void GlobalGraphHandler::rewirePathToNewNode(int path,
		int closest_waypoint_to_frontier, double new_path_length, int new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::string cons = "";
	for (auto con_path : rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to) {
		cons += std::to_string(con_path) + ",";
	}
	rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.erase(
			std::remove(
					rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.begin(),
					rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.end(),
					path),
			rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.end()); // remove path from connecting node in RRG
	_gg.paths.at(path).waypoints.erase(
			_gg.paths.at(path).waypoints.begin() + closest_waypoint_to_frontier
					+ 1, _gg.paths.at(path).waypoints.end());
	_gg.paths.at(path).waypoints.push_back(rrg.nodes.at(new_node).position);
	_gg.paths.at(path).length = new_path_length;
	_gg.paths.at(path).connecting_node = new_node;
}

void GlobalGraphHandler::tryToImproveConnectionsToOtherFrontiers(int new_node,
		int path, rrg_nbv_exploration_msgs::Graph &rrg) {
	int current_frontier = _gg.paths.at(path).frontier; // look for new connections with this frontier
	std::set<int> new_frontier_connections; // set of existing connected frontiers to the connecting node which will be reduced by the frontiers already connected to the current frontier
	std::vector<std::pair<int, double>> mergeable_frontiers; // frontier index first and distance to local graph second
	std::pair<int, double> closest_mergeable_unprunable_frontier( { -1,
			std::numeric_limits<double>::infinity() }); // frontier index first and distance to local graph second
	for (auto connected_path : rrg.nodes.at(new_node).connected_to) {
		new_frontier_connections.insert(_gg.paths.at(connected_path).frontier);
	}
	for (auto frontier_path : _gg.frontiers.at(current_frontier).paths) {
		int other_frontier =
				current_frontier == _gg.paths.at(frontier_path).frontier ?
						_gg.paths.at(frontier_path).connection :
						_gg.paths.at(frontier_path).frontier;
		if (_gg.paths.at(frontier_path).connection
				!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH
				&& new_frontier_connections.erase(other_frontier) > 0) { // remove existing connection from new frontier connections (if present, try to improve path)
			int other_frontier_path_to_local_graph = _gg.frontiers.at(
					other_frontier).paths.front();
			if ((_gg.paths.at(path).length
					+ _gg.paths.at(other_frontier_path_to_local_graph).length)
					< _gg.paths.at(frontier_path).length) { // path through new node would be shorter
				improvePathToConnectedFrontier(frontier_path, path,
						other_frontier, other_frontier_path_to_local_graph);

			}
		}
	}
	for (auto new_frontier_connection : new_frontier_connections) {
		connectFrontiers(current_frontier, new_frontier_connection, path,
				_gg.frontiers.at(new_frontier_connection).paths.front(), false); // first path of frontier is always path to local graph
	}
}

void GlobalGraphHandler::improvePathToConnectedFrontier(int frontier_path,
		int path, int other_frontier, int other_path) {
	_gg.paths.at(frontier_path).waypoints.clear();
	_gg.paths.at(frontier_path).length = _gg.paths.at(path).length
			+ _gg.paths.at(_gg.frontiers.at(other_frontier).paths.at(0)).length;
	_gg.paths.at(frontier_path).waypoints.insert(
			_gg.paths.at(frontier_path).waypoints.begin(),
			_gg.paths.at(path).waypoints.begin(),
			_gg.paths.at(path).waypoints.end() - 1); // omit last element to avoid duplicate waypoint
	_gg.paths.at(frontier_path).waypoints.insert(
			_gg.paths.at(frontier_path).waypoints.end(),
			_gg.paths.at(other_path).waypoints.rbegin(),
			_gg.paths.at(other_path).waypoints.rend());
}

bool GlobalGraphHandler::calculateNextFrontierGoal(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<int> active_frontiers;
	for (auto &frontier : _gg.frontiers) {
		if (!frontier.inactive)
			active_frontiers.push_back(frontier.index);
	}
	if (active_frontiers.size() == 1 && _auto_homing) { // only origin remains
		ROS_INFO_STREAM("Next frontier goal is the origin, going home");
		connectPathsToLocalGraphToNearestNode(rrg);
		_global_route.push_back(std::make_pair(0, 0));
		_next_global_goal = 0;
	} else if (active_frontiers.size() <= 1) {
		ROS_INFO_STREAM("No more frontier goals available");
		return false;
	} else {
		establishMissingFrontierToFrontierConnections(active_frontiers, rrg);
		connectPathsToLocalGraphToNearestNode(rrg);
		std::pair<int, int> next_frontier_with_path =
				findBestFrontierWithTspTwoOpt(active_frontiers);
		if (next_frontier_with_path.first == -1
				|| next_frontier_with_path.second == -1) {
			ROS_WARN_STREAM(
					"Unable to determine the next frontier goal with 2-Opt");
			return false;
		}
		_next_global_goal = 1;
		ROS_INFO_STREAM(
				"Next frontier goal is " << _global_route.at(_next_global_goal).first << " with path " << _global_route.at(_next_global_goal).second);
	}
	_global_path_waypoint_searcher->rebuildIndex(
			_gg.paths.at(_global_route.at(_next_global_goal).second));
	_active_paths_closest_waypoint =
			std::make_pair(_global_route.at(_next_global_goal).second,
					_gg.paths.at(_global_route.at(_next_global_goal).second).waypoints.size()
							- 1); // start at waypoint at nearest node in RRG
	return true;
}

std::pair<int, int> GlobalGraphHandler::findBestFrontierWithTspTwoOpt(
		std::vector<int> &active_frontiers) {
	std::vector<std::pair<int, int>> route; // frontier index (first) and path index from previous frontier in route (second)
	auto origin = std::find(active_frontiers.begin(), active_frontiers.end(),
			rrg_nbv_exploration_msgs::GlobalPath::ORIGIN);
	if (origin != active_frontiers.end()) { // remove origin from active frontiers (will be pushed back later when homing)
		active_frontiers.erase(origin);
	}
	route.push_back(
			std::make_pair(rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH,
					-1)); // start at robot position (local graph) with no path
	if (_auto_homing) {
		active_frontiers.push_back(
				rrg_nbv_exploration_msgs::GlobalPath::ORIGIN); // origin as last frontier
	}
	route.push_back(
			std::make_pair(active_frontiers.front(),
					_gg.frontiers.at(active_frontiers.front()).paths.front())); // add first active frontier and path to local graph to route
	for (int i = 1; i < active_frontiers.size(); i++) { // add frontier index and index of path from previous frontier to route except for first entry
		int path_index = findPathToNextFrontier(active_frontiers.at(i - 1),
				active_frontiers.at(i));
		if (path_index == -1) {
			return std::make_pair(-1, -1);
		}
		route.push_back(std::make_pair(active_frontiers.at(i), path_index));
	}
	bool improved = true;
	while (improved) {
		improved = iterateOverTwoOptSwaps(route);
	}
	_global_route = route;
	return std::make_pair(route.at(1).first, route.at(1).second);
}

bool GlobalGraphHandler::iterateOverTwoOptSwaps(
		std::vector<std::pair<int, int>> &route) {
	double best_distance = calculateRouteLength(route);
	for (int i = 1; i < route.size() - 1; i++) { // omit first frontier (robot position/local graph)
		for (int k = i + 1; k < route.size() - (_auto_homing ? 1 : 0); k++) { // omit last frontier (origin) when auto homing
			std::vector<std::pair<int, int>> new_route;
			if (twoOptSwap(route, new_route, i, k)) { // if swap resulted in traversable route
				double new_distance = calculateRouteLength(new_route);
				if (new_distance < best_distance) {
					route = new_route;
					best_distance = new_distance;
					return true;
				}
			}
		}
	}
	return false;
}

int GlobalGraphHandler::findPathToNextFrontier(int current_frontier,
		int next_frontier) {
	int path_index = -1;
	if (current_frontier == rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH)
		path_index = _gg.frontiers.at(next_frontier).paths.front();
	else if (next_frontier == rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH)
		path_index = _gg.frontiers.at(current_frontier).paths.front();
	else {
		for (auto path : _gg.frontiers.at(current_frontier).paths) {
			if (!_gg.paths.at(path).inactive
					&& (_gg.paths.at(path).frontier == next_frontier
							|| _gg.paths.at(path).connection == next_frontier)) {
				path_index = path;
				break;
			}
		}
	}
	return path_index;
}

double GlobalGraphHandler::calculateRouteLength(
		std::vector<std::pair<int, int>> &route) {
	double distance = 0.0;
	for (int i = 1; i < route.size(); i++) { // omit first frontier (local graph) in route
		distance += _gg.paths.at(route.at(i).second).length;
	}
	return distance;
}

bool GlobalGraphHandler::twoOptSwap(std::vector<std::pair<int, int>> &route,
		std::vector<std::pair<int, int>> &new_route, int i, int k) {
	for (int j = 0; j < i; j++) {
		new_route.push_back(route.at(j));
	}
	for (int l = k; l >= i; l--) {
		int path_index = findPathToNextFrontier(new_route.back().first,
				route.at(l).first);
		new_route.push_back(route.at(l));
		new_route.back().second = path_index;
	}
	for (int m = k + 1; m < route.size(); m++) {
		new_route.push_back(route.at(m));
	}
	return true;
}

void GlobalGraphHandler::establishMissingFrontierToFrontierConnections(
		std::vector<int> active_frontiers,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<ShortestFrontierConnectionStruct> frontier_connections;
	for (auto &frontier : _gg.frontiers) { // establish all missing connection between frontiers through the RRG
		if (!frontier.inactive
				&& frontier.paths_counter < active_frontiers.size()) { // one path to every other frontier + 1 to RRG required
			std::set<int> missing_frontiers = getMissingFrontierConnections(
					active_frontiers, frontier);
			int frontier_connecting_node =
					_gg.paths.at(frontier.paths.front()).connecting_node;
			double max_distance_threshold = 0; // maximum path length threshold when searching for connections
			std::vector<std::pair<int, int>> missing_frontier_connections; // missing frontier index (first) and connecting node of missing frontier to local graph (second)
			for (auto missing_frontier : missing_frontiers) { // build the missing paths
				if (frontier.index > missing_frontier) { // only build paths where this frontier will be frontier in path
					int missing_frontier_connecting_node =
							_gg.paths.at(
									_gg.frontiers.at(missing_frontier).paths.front()).connecting_node;
					bool found_existing_connection = false;
					for (auto frontier_connection : frontier_connections) { //check if a connection between the connecting nodes of both frontiers was already found and reuse it
						if (frontier_connection.connecting_node_one
								== std::max(frontier_connecting_node,
										missing_frontier_connecting_node)
								&& frontier_connection.connecting_node_two
										== std::min(frontier_connecting_node,
												missing_frontier_connecting_node)) {
							found_existing_connection = true;
							buildMissingPathBetweenFrontiers(frontier,
									missing_frontier,
									frontier_connection.path_length,
									frontier_connection.path, rrg);
							break;
						}
					}
					if (!found_existing_connection) {
						missing_frontier_connections.push_back(
								std::make_pair(missing_frontier,
										missing_frontier_connecting_node));
						findShortestPathThroughMutualNode(
								frontier_connecting_node,
								missing_frontier_connecting_node,
								max_distance_threshold, rrg);
					}
				}
			}
			if (!missing_frontier_connections.empty()) { // only find shortest routes if a connection is missing
				std::vector<ShortestFrontierConnectionStruct> local_paths;
				std::map<int, int> missing_frontier_local_path_map =
						_graph_path_calculator->findShortestRoutes(rrg,
								frontier_connecting_node,
								missing_frontier_connections, local_paths,
								max_distance_threshold);
				frontier_connections.insert(frontier_connections.end(),
						local_paths.begin(), local_paths.end()); // add local paths to frontier connections
				for (auto missing_frontier_local_path : missing_frontier_local_path_map) { // iterate over map (first=missing frontier index, second=local path index)
					buildMissingPathBetweenFrontiers(frontier,
							missing_frontier_local_path.first,
							local_paths.at(missing_frontier_local_path.second).path_length,
							local_paths.at(missing_frontier_local_path.second).path,
							rrg);
				}
			}
		}
	}
}

std::set<int> GlobalGraphHandler::getMissingFrontierConnections(
		std::vector<int> &active_frontiers,
		rrg_nbv_exploration_msgs::GlobalFrontier &frontier) {
	std::set<int> missing_frontiers(active_frontiers.begin(),
			active_frontiers.end()); // one path to every other frontier + 1 to RRG required
	missing_frontiers.erase(frontier.index);
	for (int path : frontier.paths) {
		if (_gg.paths.at(path).connection
				!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH)
			missing_frontiers.erase(
					frontier.index == _gg.paths.at(path).frontier ?
							_gg.paths.at(path).connection :
							_gg.paths.at(path).frontier);
	}
	return missing_frontiers;
}

void GlobalGraphHandler::findShortestPathThroughMutualNode(
		int frontier_connecting_node, int missing_frontier_connecting_node,
		double &max_distance_threshold, rrg_nbv_exploration_msgs::Graph &rrg) {
	int first_mutual_node_in_path = rrg.nearest_node;
	double minimum_local_path_length =
			rrg.nodes.at(frontier_connecting_node).distance_to_robot
					+ rrg.nodes.at(missing_frontier_connecting_node).distance_to_robot;
	std::vector<int> mutual_node_local_path; // the shortest possible path between the two frontiers using a mutual node in both connecting node's paths to robot
	for (auto rit =
			rrg.nodes.at(frontier_connecting_node).path_to_robot.rbegin();
			rit
					!= std::prev(
							rrg.nodes.at(frontier_connecting_node).path_to_robot.rend());
			++rit) { // reverse end is connecting node
		auto it =
				std::find(
						std::next(
								rrg.nodes.at(missing_frontier_connecting_node).path_to_robot.begin()), // first node is connecting node
						rrg.nodes.at(missing_frontier_connecting_node).path_to_robot.end(),
						*rit); // find occurence of node in both paths
		if (it
				!= rrg.nodes.at(missing_frontier_connecting_node).path_to_robot.end()) { // found mutual node in paths
			first_mutual_node_in_path = *it;
			mutual_node_local_path.insert(mutual_node_local_path.end(),
					rrg.nodes.at(frontier_connecting_node).path_to_robot.rbegin(),
					rit);
			mutual_node_local_path.insert(mutual_node_local_path.end(), it,
					rrg.nodes.at(missing_frontier_connecting_node).path_to_robot.end());
			std::string rempa = "";
			for (int rem : mutual_node_local_path) {
				rempa += std::to_string(rem) + ",";
			}
			double length = 0;
			for (int i = 0; i < mutual_node_local_path.size() - 1; i++) { // iterate over edges in remaining path to calculate length
				int edge = _graph_path_calculator->findExistingEdge(rrg,
						mutual_node_local_path.at(i),
						mutual_node_local_path.at(i + 1));
				if (edge != -1) {
					length += rrg.edges.at(edge).length;
				} else {
					ROS_WARN_STREAM(
							"Unable to calculate minimum local path, node " << mutual_node_local_path.at(i) << " has no edge to " << mutual_node_local_path.at(i + 1));
					length = 0;
					break;
				}
			}
			if (length > 0) {
				minimum_local_path_length = length;
				break;
			}
		}
	}
	if (mutual_node_local_path.empty()) {
		max_distance_threshold = std::numeric_limits<double>::infinity(); // cannot use distance threshold
	}
	max_distance_threshold = std::max(max_distance_threshold,
			minimum_local_path_length);
}

void GlobalGraphHandler::buildMissingPathBetweenFrontiers(
		rrg_nbv_exploration_msgs::GlobalFrontier_<std::allocator<void>> &frontier,
		int missing_frontier, double path_length, std::vector<int> &local_path,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	rrg_nbv_exploration_msgs::GlobalPath path_between_frontiers;
	path_between_frontiers.index = availablePathIndex();
	path_between_frontiers.frontier = frontier.index;
	path_between_frontiers.connection = missing_frontier;
	path_between_frontiers.waypoints.insert(
			path_between_frontiers.waypoints.begin(),
			_gg.paths.at(_gg.frontiers.at(frontier.index).paths.front()).waypoints.begin(),
			_gg.paths.at(_gg.frontiers.at(frontier.index).paths.front()).waypoints.end());
	for (int i = 1; i < local_path.size() - 1; i++) { // omit begin and end, connecting nodes are already present in path
		path_between_frontiers.waypoints.push_back(
				rrg.nodes.at(local_path.at(i)).position);
	}
	path_between_frontiers.waypoints.insert(
			path_between_frontiers.waypoints.end(),
			_gg.paths.at(_gg.frontiers.at(missing_frontier).paths.front()).waypoints.rbegin(),
			_gg.paths.at(_gg.frontiers.at(missing_frontier).paths.front()).waypoints.rend());
	path_between_frontiers.length =
			_gg.paths.at(_gg.frontiers.at(frontier.index).paths.front()).length
					+ path_length
					+ _gg.paths.at(
							_gg.frontiers.at(missing_frontier).paths.front()).length;
	path_between_frontiers.connecting_node = -1;
	_gg.frontiers.at(frontier.index).paths.push_back(
			path_between_frontiers.index);
	_gg.frontiers.at(frontier.index).paths_counter++;
	_gg.frontiers.at(missing_frontier).paths.push_back(
			path_between_frontiers.index);
	_gg.frontiers.at(missing_frontier).paths_counter++;
	insertPathInGg(path_between_frontiers);
}

void GlobalGraphHandler::connectPathsToLocalGraphToNearestNode(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto &path : _gg.paths) { // connect all frontiers directly to the nearest node to the robot
		if (!path.inactive
				&& path.connection
						== rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) { // add path to robot from connecting node
			for (int i = rrg.nodes.at(path.connecting_node).path_to_robot.size()
					- 2; i >= 0; i--) { // skip node itself, as it already is in waypoints
				path.waypoints.push_back(
						rrg.nodes.at(
								rrg.nodes.at(path.connecting_node).path_to_robot.at(
										i)).position);
			}
			path.length += rrg.nodes.at(path.connecting_node).distance_to_robot;
			path.connecting_node = rrg.nearest_node;
		}
	}
}

bool GlobalGraphHandler::checkIfNextFrontierWithPathIsValid() {
	return _next_global_goal >= 0 && _next_global_goal < _global_route.size()
			&& _global_route.at(_next_global_goal).first >= 0
			&& _global_route.at(_next_global_goal).first < _gg.frontiers_counter
			&& _global_route.at(_next_global_goal).second >= 0
			&& _global_route.at(_next_global_goal).second < _gg.paths_counter;
}

bool GlobalGraphHandler::getFrontierGoal(geometry_msgs::Point &goal,
		double &yaw, geometry_msgs::Point &robot_pos) {
	if (checkIfNextFrontierWithPathIsValid()) {
		goal =
				_gg.frontiers.at(_global_route.at(_next_global_goal).first).viewpoint;
		if (_gg.paths.at(_global_route.at(_next_global_goal).second).waypoints.size()
				> 1) {
			yaw =
					(int) (atan2(
							goal.y
									- _gg.paths.at(
											_global_route.at(_next_global_goal).second).waypoints.at(
											1).y,
							goal.x
									- _gg.paths.at(
											_global_route.at(_next_global_goal).second).waypoints.at(
											1).x) * 180 / M_PI);
		} else { // frontier directly next to robot, define yaw from robot position
			yaw = (int) (atan2(goal.y - robot_pos.y, goal.x - robot_pos.x) * 180
					/ M_PI);
		}
		return true;
	} else {
		return false;
	}
}

bool GlobalGraphHandler::getFrontierPath(
		std::vector<geometry_msgs::PoseStamped> &path,
		geometry_msgs::Point &robot_pos) {
	if (checkIfNextFrontierWithPathIsValid()) {
		std::vector<geometry_msgs::Point> waypoints;
		if (_previous_global_goal_failed) { // go back to former local graph to start navigation to next frontier from there
			waypoints.insert(waypoints.end(),
					_gg.paths.at(_global_route.at(_next_global_goal - 1).second).waypoints.rbegin(),
					_gg.paths.at(_global_route.at(_next_global_goal - 1).second).waypoints.rend());
			waypoints.insert(waypoints.end(),
					_gg.paths.at(_global_route.at(_next_global_goal).second).waypoints.begin(),
					_gg.paths.at(_global_route.at(_next_global_goal).second).waypoints.end());
		} else {
			waypoints.insert(waypoints.end(),
					_gg.paths.at(_global_route.at(_next_global_goal).second).waypoints.begin(),
					_gg.paths.at(_global_route.at(_next_global_goal).second).waypoints.end());
		}
		_graph_path_calculator->getNavigationPath(path, waypoints, robot_pos,
				_active_paths_closest_waypoint.second);
		return true;
	} else {
		return false;
	}
}

std::vector<int> GlobalGraphHandler::frontierReached(
		geometry_msgs::Point &position) {
	std::vector<int> connected_paths;
	if (checkIfNextFrontierWithPathIsValid()) {
		int frontier = _global_route.at(_next_global_goal).first;
		position = _gg.frontiers.at(frontier).viewpoint;
		_next_global_goal = -1;
		_global_route.clear();
		_previous_global_goal_failed = false;
		std::set<int> pruned_frontiers;
		std::set<int> pruned_paths;
		pruned_frontiers.insert(frontier);
		for (auto path : _gg.frontiers.at(frontier).paths) {
			pruned_paths.insert(path);
			overwritePathToLocalGraph(path, frontier, connected_paths);
		}
		if (pruned_frontiers.size() > 0) {
			handlePrunedFrontiers(pruned_frontiers);
			handlePrunedPaths(pruned_paths);
		}
	}
	return connected_paths;
}

bool GlobalGraphHandler::frontierFailed() {
	if (_next_global_goal >= _global_route.size() - 1) { // last frontier in route, exploration finished
		return true;
	} else {
		_previous_global_goal_failed = true;
		_next_global_goal++;
		ROS_INFO_STREAM(
				"Frontier " << _global_route.at(_next_global_goal - 1).first << " failed, try next frontier " << _global_route.at(_next_global_goal).first);
		return false;
	}
}

void GlobalGraphHandler::overwritePathToLocalGraph(int path, int frontier,
		std::vector<int> &connected_paths) {
	if (_gg.paths.at(path).connection
			!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) {
		int other_frontier =
				_gg.paths.at(path).frontier == frontier ?
						_gg.paths.at(path).connection :
						_gg.paths.at(path).frontier;
		int other_frontiers_path_to_local_graph = _gg.frontiers.at(
				other_frontier).paths.front();
		// overwrite path between other frontier and local graph with details from the path between those two
		_gg.paths.at(other_frontiers_path_to_local_graph).connecting_node = 0; // root at new RRG
		_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.clear();
		if (other_frontier > frontier) {
			_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.insert(
					_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.end(),
					_gg.paths.at(path).waypoints.begin(),
					_gg.paths.at(path).waypoints.end());
		} else { // path was from frontier to other frontier, must be reversed
			_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.insert(
					_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.end(),
					_gg.paths.at(path).waypoints.rbegin(),
					_gg.paths.at(path).waypoints.rend());
		}
		_gg.paths.at(other_frontiers_path_to_local_graph).length = _gg.paths.at(
				path).length;
		connected_paths.push_back(other_frontiers_path_to_local_graph);
	}
}

bool GlobalGraphHandler::updateClosestWaypoint(
		geometry_msgs::Point &robot_pos) {
	double min_distance;
	int nearest_node;
	_global_path_waypoint_searcher->findNearestNeighbour(robot_pos,
			min_distance, nearest_node);
	if (nearest_node != _active_paths_closest_waypoint.second) {
		_active_paths_closest_waypoint.second = nearest_node;
		if (nearest_node == 0) { // frontier reached
			_active_paths_closest_waypoint = std::make_pair(-1, -1);
			return true;
		} else if (_previous_global_goal_failed
				&& nearest_node
						== _gg.paths.at(_active_paths_closest_waypoint.first).waypoints.size()
								- 1) { // reached start of path, transfer to path to next frontier
			_active_paths_closest_waypoint =
					std::make_pair(_global_route.at(_next_global_goal).second,
							_gg.paths.at(
									_global_route.at(_next_global_goal).second).waypoints.size()
									- 1);
			_global_path_waypoint_searcher->rebuildIndex(
					_gg.paths.at(_global_route.at(_next_global_goal).second));
			_previous_global_goal_failed = false;
		}
	}
	return false;
}

void GlobalGraphHandler::dynamicReconfigureCallback(
		rrg_nbv_exploration::GraphConstructorConfig &config, uint32_t level) {
	_local_graph_radius = std::max(config.local_graph_radius,
			2 * _robot_radius);
}

} /* namespace rrg_nbv_exploration */
