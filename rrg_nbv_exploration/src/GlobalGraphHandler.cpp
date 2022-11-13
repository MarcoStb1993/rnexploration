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
	_half_connection_box_distance_thres = sqrt(
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
	_global_connection_waypoint_searcher.reset(new GlobalGraphWaypointSearcher());
}

GlobalGraphHandler::~GlobalGraphHandler() {
}

void GlobalGraphHandler::initialize(rrg_nbv_exploration_msgs::Node &root,
		std::shared_ptr<GraphPathCalculator> graph_connection_calculator,
		std::shared_ptr<GraphSearcher> graph_searcher,
		std::shared_ptr<CollisionChecker> collision_checker) {
	_gg.targets.clear();
	_gg.connections.clear();
	_graph_connection_calculator = std::move(graph_connection_calculator);
	_graph_searcher = std::move(graph_searcher);
	_collision_checker = std::move(collision_checker);
	_gg.header.frame_id = "map";
	_gg.ns = "globalgraph";
	_gg.targets_counter = 0;
	_gg.connections_counter = 0;
	rrg_nbv_exploration_msgs::GlobalTarget target;
	target.index = _gg.targets_counter++;
	target.viewpoint = root.position;
	rrg_nbv_exploration_msgs::GlobalConnection connection;
	connection.index = _gg.connections_counter++;
	connection.first_target = rrg_nbv_exploration_msgs::GlobalConnection::ORIGIN;
	connection.second_target = rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH;
	connection.connecting_node = root.index;
	connection.waypoints.push_back(root.position);
	connection.length = 0;
	target.connections.push_back(connection.index);
	target.connections_counter++;
	_gg.targets.push_back(target);
	_gg.connections.push_back(connection);
	root.connected_to.push_back(connection.index);
	_available_targets.clear();
	_available_connections.clear();
	_global_route.clear();
	_next_global_goal = -1;
	_previous_global_goal_failed = -1;
	_active_connections_closest_waypoint = std::make_pair(-1, -1);
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
		int edge = _graph_connection_calculator->findExistingEdge(rrg,
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
		const rrg_nbv_exploration_msgs::GlobalTarget &target) {
	if (!_available_targets.empty()) {
		_gg.targets.at(*_available_targets.begin()) = target;
		_available_targets.erase(_available_targets.begin());
	} else {
		_gg.targets.push_back(target);
		_gg.targets_counter++;
	}
}

void GlobalGraphHandler::insertPathInGg(
		const rrg_nbv_exploration_msgs::GlobalConnection &connection_between_targets) {
	if (!_available_connections.empty()) {
		_gg.connections.at(*_available_connections.begin()) = connection_between_targets;
		_available_connections.erase(_available_connections.begin());
	} else {
		_gg.connections.push_back(connection_between_targets);
		_gg.connections_counter++;
	}
}

void GlobalGraphHandler::addFrontier(int node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	rrg_nbv_exploration_msgs::GlobalTarget target;
	rrg_nbv_exploration_msgs::GlobalConnection connection;
	target.index = availableFrontierIndex();
	target.viewpoint = rrg.nodes.at(node).position;
	target.merged_distance = 0.0;
	connection.second_target = rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH;
	connection.waypoints.push_back(rrg.nodes.at(node).position);
	if (!getConnectingNode(node, rrg, connection.waypoints, connection.connecting_node,
			connection.length)) {
		ROS_WARN_STREAM(
				"Unable to add target for node " << node << ", found no connection to the RRG");
		return;
	}
	if (tryToMergeAddedFrontiers(node, connection, rrg, target)) { // if new target is merged into existing
		return;
	}
	connection.index = availablePathIndex();
	connection.first_target = target.index;
	target.connections.push_back(connection.index);
	target.connections_counter++;
	insertFrontierInGg(target);
	insertPathInGg(connection);
	for (auto connected_connection : rrg.nodes.at(node).connected_to) { // new target is in connection of existing targets
		connectFrontiers(target.index, _gg.connections.at(connected_connection).first_target,
				connection.index, connected_connection, true);
	}
	for (auto connected_connection : rrg.nodes.at(connection.connecting_node).connected_to) { // connecting node is in connection of existing targets, connect targets
		connectFrontiers(target.index, _gg.connections.at(connected_connection).first_target,
				connection.index, connected_connection, false);
	}
	rrg.nodes.at(connection.connecting_node).connected_to.push_back(connection.index);
	_global_graph_searcher->rebuildIndex(_gg);
}

void GlobalGraphHandler::continuePath(int node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<geometry_msgs::Point> additional_waypoints;
	int connecting_node;
	double length = 0;
	if (!getConnectingNode(node, rrg, additional_waypoints, connecting_node,
			length)) {
		ROS_ERROR_STREAM(
				"Unable to continue connections at node " << node << ", found no connection to the RRG");
		return;
	}
	std::map<int, int> connecting_node_targets; // target index (first), connection to connected node index (second)
	for (auto connected_connection : rrg.nodes.at(connecting_node).connected_to) { // get list of already present connection connections at new connected node in local graph
		connecting_node_targets.insert(
				{ _gg.connections.at(connected_connection).first_target, connected_connection });
	}
	std::vector<int> connected_connections = rrg.nodes.at(node).connected_to; //create copy because connected connections can be pruned in the for loop
	for (int connection : connected_connections) {
		if (connection < _gg.connections_counter && !_gg.connections.at(connection).inactive
				&& _gg.connections.at(connection).connecting_node == node) { //check if the connection was not pruned (and replaced)
			_gg.connections.at(connection).waypoints.insert(
					_gg.connections.at(connection).waypoints.end(),
					additional_waypoints.begin(), additional_waypoints.end()); // add additional waypoints to connection
			_gg.connections.at(connection).connecting_node = connecting_node;
			_gg.connections.at(connection).length += length;
			rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.push_back(
					connection);
			if (connecting_node_targets.size() > 0) {
				int current_target = _gg.connections.at(connection).first_target; // look for new connections with this target
				std::set<int> new_target_connections;	// set of existing connected targets to the connecting node which will be reduced by the targets already connected to the current target
				for (auto elem : connecting_node_targets) {
					new_target_connections.insert(elem.first);
				}
				for (auto target_connection : _gg.targets.at(current_target).connections) {
					if (_gg.connections.at(target_connection).second_target
							!= rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH) { // connected to another target
						new_target_connections.erase(
								current_target // current target can be target or connection at connecting connection
								== _gg.connections.at(target_connection).first_target ?
										_gg.connections.at(target_connection).second_target :
										_gg.connections.at(target_connection).first_target); // remove from new targets (if present)
					}
				}
				if (current_target
						!= rrg_nbv_exploration_msgs::GlobalConnection::ORIGIN
						&& tryToMergeContinuedPaths(current_target, connection,
								connecting_node_targets,
								new_target_connections, rrg)) //if current target was pruned
					continue;
				for (auto new_target_connection : new_target_connections) { //remaining missing target connections
					connectFrontiers(current_target, new_target_connection,
							connection,
							connecting_node_targets.at(
									new_target_connection), false);
				}
			}
		}
	}
}

bool GlobalGraphHandler::tryToMergeContinuedPaths(int current_target,
		int connection, std::map<int, int> &connecting_node_targets,
		std::set<int> &new_target_connections,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	bool pruned_current_target = false;
	std::vector<MergeableFrontierStruct> mergeable_targets;
	MergeableFrontierStruct closest_mergeable_unprunable_target(-1,
			std::numeric_limits<double>::infinity(), 0);
	for (auto new_target_connection : new_target_connections) { //check if can be merged
		checkFrontierMergeability(_gg.connections.at(connection),
				connecting_node_targets.at(new_target_connection),
				_gg.targets.at(current_target),
				closest_mergeable_unprunable_target, mergeable_targets);
	}
	std::set<int> pruned_targets;
	std::set<int> pruned_connections;
	if (closest_mergeable_unprunable_target.target != -1) { //merge this target into closest mergeable unprunable target
		_gg.targets.at(closest_mergeable_unprunable_target.target).merged_distance =
				std::max(
						_gg.targets.at(current_target).merged_distance
								+ closest_mergeable_unprunable_target.distance_between_targets,
						_gg.targets.at(
								closest_mergeable_unprunable_target.target).merged_distance);
		addFrontierToBePruned(current_target, pruned_targets, pruned_connections,
				rrg);
		pruned_current_target = true;
	} else if (mergeable_targets.size()) {
		int remaining_target = current_target;
		double shortest_connection_length = _gg.connections.at(connection).length;
		std::set<std::pair<double, int>> merged_distances_to_targets; //merged distance to target (first) and target index (second) ordered by merged distance
		merged_distances_to_targets.insert(
				std::make_pair(
						_gg.targets.at(current_target).merged_distance,
						current_target));
		double distance_current_target_to_remaining_target = 0;
		for (auto mergeable_target : mergeable_targets) { // find remaining target
			merged_distances_to_targets.insert(
					std::make_pair(
							_gg.targets.at(mergeable_target.target).merged_distance
									+ mergeable_target.distance_between_targets,
							mergeable_target.target));
			if (mergeable_target.connection_length_to_local_graph
					<= shortest_connection_length) {
				if (remaining_target != current_target) { // not in graph, cannot be pruned
					addFrontierToBePruned(remaining_target, pruned_targets,
							pruned_connections, rrg);
					removeFrontierFromConnectableFrontierList(
							remaining_target, connecting_node_targets,
							new_target_connections);
				}
				remaining_target = mergeable_target.target;
				shortest_connection_length =
						mergeable_target.connection_length_to_local_graph;
				distance_current_target_to_remaining_target =
						mergeable_target.distance_between_targets;
			} else {
				addFrontierToBePruned(mergeable_target.target,
						pruned_targets, pruned_connections, rrg);
				removeFrontierFromConnectableFrontierList(
						mergeable_target.target, connecting_node_targets,
						new_target_connections);
			}
		}
		if (remaining_target != current_target) { // don't add target, it was merged into existing
			double max_relevant_merged_distance = 0;
			if (merged_distances_to_targets.rbegin()->second
					!= remaining_target) { //check if max merged distance is not merged distance to remaining target, because this would add distance_current_target_to_remaining_target twice
				max_relevant_merged_distance =
						merged_distances_to_targets.rbegin()->first
								+ distance_current_target_to_remaining_target;
			} else { //otherwise take second merged distance which is to another target
				max_relevant_merged_distance = std::next(
						merged_distances_to_targets.rbegin())->first
						+ distance_current_target_to_remaining_target;
			}
			if (max_relevant_merged_distance > _local_graph_radius) { //if new merged distance violates the constraint, prune the remaining target and keep the current target
				addFrontierToBePruned(remaining_target, pruned_targets,
						pruned_connections, rrg);
				removeFrontierFromConnectableFrontierList(remaining_target,
						connecting_node_targets, new_target_connections);
				_gg.targets.at(current_target).merged_distance =
						merged_distances_to_targets.rbegin()->first;
			} else { //merge like all other targets were merged into current target and then into remaining target
				addFrontierToBePruned(current_target, pruned_targets,
						pruned_connections, rrg);
				_gg.targets.at(remaining_target).merged_distance = std::max(
						max_relevant_merged_distance,
						_gg.targets.at(remaining_target).merged_distance);
				pruned_current_target = true;
			}
		} else {
			_gg.targets.at(current_target).merged_distance =
					merged_distances_to_targets.rbegin()->first;
		}
	}
	if (pruned_targets.size() > 0) {
		handlePrunedFrontiers(pruned_targets);
		handlePrunedPaths(pruned_connections);
	}
	return pruned_current_target;
}

void GlobalGraphHandler::removeFrontierFromConnectableFrontierList(int target,
		std::map<int, int> &connecting_node_targets,
		std::set<int> &new_target_connections) {
	auto it = connecting_node_targets.find(target);
	if (it != connecting_node_targets.end()) {
		connecting_node_targets.erase(it);
	}
	new_target_connections.erase(target);
}

bool GlobalGraphHandler::tryToMergeAddedFrontiers(int node,
		rrg_nbv_exploration_msgs::GlobalConnection &connection,
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::GlobalTarget &target) {
	bool pruned_added_target = false;
	if (!rrg.nodes.at(node).connected_to.empty()
			|| !rrg.nodes.at(connection.connecting_node).connected_to.empty()) {
		std::vector<MergeableFrontierStruct> mergeable_targets;
		MergeableFrontierStruct closest_mergeable_unprunable_target(-1,
				std::numeric_limits<double>::infinity(), 0);
		for (auto connected_connection : rrg.nodes.at(node).connected_to) {
			checkFrontierMergeability(connection, connected_connection, target,
					closest_mergeable_unprunable_target, mergeable_targets,
					false);
		}
		for (auto connected_connection : rrg.nodes.at(connection.connecting_node).connected_to) {
			checkFrontierMergeability(connection, connected_connection, target,
					closest_mergeable_unprunable_target, mergeable_targets);
		}
		if (closest_mergeable_unprunable_target.target != -1) { //merge this target into closest mergeable unprunable target
			_gg.targets.at(closest_mergeable_unprunable_target.target).merged_distance =
					std::max(
							target.merged_distance
									+ closest_mergeable_unprunable_target.distance_between_targets,
							_gg.targets.at(
									closest_mergeable_unprunable_target.target).merged_distance);
			pruned_added_target = true;
		} else if (mergeable_targets.size()) {
			std::set<int> pruned_targets;
			std::set<int> pruned_connections;
			int remaining_target = target.index;
			double shortest_connection_length = connection.length;
			std::set<std::pair<double, int>> merged_distances_to_targets; //merged distance to target (first) and target index (second) ordered by merged distance
			merged_distances_to_targets.insert(
					std::make_pair(target.merged_distance, target.index));
			double distance_current_target_to_remaining_target = 0;
			for (auto mergeable_target : mergeable_targets) { // find remaining target
				merged_distances_to_targets.insert(
						std::make_pair(
								_gg.targets.at(mergeable_target.target).merged_distance
										+ mergeable_target.distance_between_targets,
								mergeable_target.target));
				if (mergeable_target.connection_length_to_local_graph
						<= shortest_connection_length) {
					if (remaining_target != target.index) { // not in graph, cannot be pruned
						addFrontierToBePruned(remaining_target,
								pruned_targets, pruned_connections, rrg);
					}
					remaining_target = mergeable_target.target;
					shortest_connection_length =
							mergeable_target.connection_length_to_local_graph;
					distance_current_target_to_remaining_target =
							mergeable_target.distance_between_targets;
				} else {
					addFrontierToBePruned(mergeable_target.target,
							pruned_targets, pruned_connections, rrg);
				}
			}
			if (remaining_target != target.index) { // don't add target, it was merged into existing
				double max_relevant_merged_distance = 0;
				if (merged_distances_to_targets.rbegin()->second
						== remaining_target) { //check if max merged distance is not merged distance to remaining target, because this would add distance_current_target_to_remaining_target twice
					max_relevant_merged_distance =
							merged_distances_to_targets.rbegin()->first
									+ distance_current_target_to_remaining_target;
				} else { //otherwise take second merged distance which is to another target
					max_relevant_merged_distance = std::next(
							merged_distances_to_targets.rbegin())->first
							+ distance_current_target_to_remaining_target;
				}
				if (max_relevant_merged_distance > _local_graph_radius) { //if new merged distance violates the constraint, prune the remaining target and keep the added target
					addFrontierToBePruned(remaining_target, pruned_targets,
							pruned_connections, rrg);
					target.merged_distance =
							merged_distances_to_targets.rbegin()->first;
				} else { //merge like all other targets were merged into current target and then into remaining target
					_gg.targets.at(remaining_target).merged_distance =
							std::max(max_relevant_merged_distance,
									_gg.targets.at(remaining_target).merged_distance);
					pruned_added_target = true;
				}
			} else {
				target.merged_distance =
						merged_distances_to_targets.rbegin()->first;
			}
			if (pruned_targets.size() > 0) {
				handlePrunedFrontiers(pruned_targets);
				handlePrunedPaths(pruned_connections);
			}
			if (!pruned_added_target) {
				target.index = availableFrontierIndex();
			}
		}
	}
	return pruned_added_target;
}

void GlobalGraphHandler::checkFrontierMergeability(
		rrg_nbv_exploration_msgs::GlobalConnection &connection_one, int connection_two,
		rrg_nbv_exploration_msgs::GlobalTarget &target_one,
		MergeableFrontierStruct &closest_mergeable_unprunable_target,
		std::vector<MergeableFrontierStruct> &mergeable_targets,
		double add_connection_one_length) {
	double combined_length = _gg.connections.at(connection_two).length
			+ (add_connection_one_length ? connection_one.length : 0);
	double distance = _graph_connection_calculator->distance2d(target_one.viewpoint,
			_gg.targets.at(_gg.connections.at(connection_two).first_target).viewpoint);
	if (_gg.connections.at(connection_two).first_target
			!= rrg_nbv_exploration_msgs::GlobalConnection::ORIGIN
			&& combined_length <= 2 * _local_graph_radius
			&& distance <= _local_graph_radius) {
		double target_one_new_merged_distance = target_one.merged_distance
				+ distance;
		double target_two_new_merged_distance = _gg.targets.at(
				_gg.connections.at(connection_two).first_target).merged_distance + distance;
		if (target_one_new_merged_distance <= _local_graph_radius
				|| target_two_new_merged_distance <= _local_graph_radius) {
			if (target_two_new_merged_distance > _local_graph_radius) { //target one won't have the targets merged into target two in its local graph radius, therefore target two is unpruneable
				if (combined_length
						< closest_mergeable_unprunable_target.connection_length_to_local_graph) {
					closest_mergeable_unprunable_target =
							MergeableFrontierStruct(
									_gg.connections.at(connection_two).first_target,
									_gg.connections.at(connection_two).length, distance);
				}
			} else {
				mergeable_targets.emplace_back(
						_gg.connections.at(connection_two).first_target,
						_gg.connections.at(connection_two).length, distance);
			}
		}
	}
}

void GlobalGraphHandler::connectFrontiers(int target_one, int target_two,
		int connection_one, int connection_two, bool connection_at_target) {
	int target;
	int connection;
	int target_connection;
	int connection_connection;
	bool skip_target_connection = false;
	bool skip_connection_connection = false;
	if (target_one > target_two) { // higher index target is always target, the other one connection, connection always starts at target
		target = target_one;
		connection = target_two;
		target_connection = connection_one;
		connection_connection = connection_two;
		if (connection_at_target)
			skip_target_connection = true;
	} else {
		target = target_two;
		connection = target_one;
		target_connection = connection_two;
		connection_connection = connection_one;
		if (connection_at_target)
			skip_connection_connection = true;
	}
	rrg_nbv_exploration_msgs::GlobalConnection connection_between_targets;
	connection_between_targets.index = availablePathIndex();
	connection_between_targets.first_target = target;
	connection_between_targets.second_target = connection;
	connection_between_targets.length = 0;
	if (!skip_target_connection) {
		connection_between_targets.waypoints.insert(
				connection_between_targets.waypoints.begin(),
				_gg.connections.at(target_connection).waypoints.begin(),
				_gg.connections.at(target_connection).waypoints.end() - 1); // omit last element to avoid duplicate waypoint
		connection_between_targets.length += _gg.connections.at(target_connection).length;
	}
	if (!skip_connection_connection) {
		connection_between_targets.waypoints.insert(
				connection_between_targets.waypoints.end(),
				_gg.connections.at(connection_connection).waypoints.rbegin(),
				_gg.connections.at(connection_connection).waypoints.rend());
		connection_between_targets.length += _gg.connections.at(connection_connection).length;
	}
	connection_between_targets.connecting_node = -1;
	_gg.targets.at(target).connections.push_back(connection_between_targets.index);
	_gg.targets.at(target).connections_counter++;
	_gg.targets.at(connection).connections.push_back(connection_between_targets.index);
	_gg.targets.at(connection).connections_counter++;
	insertPathInGg(connection_between_targets);
}

int GlobalGraphHandler::availableFrontierIndex() {
	if (!_available_targets.empty()) {
		return *_available_targets.begin();
	} else {
		return _gg.targets_counter;
	}
}

int GlobalGraphHandler::availablePathIndex() {
	if (!_available_connections.empty()) {
		return *_available_connections.begin();
	} else {
		return _gg.connections_counter;
	}
}

void GlobalGraphHandler::addFrontierToBePruned(int target_to_prune,
		std::set<int> &pruned_targets, std::set<int> &pruned_connections,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	pruned_targets.insert(target_to_prune);
	for (auto connection : _gg.targets.at(target_to_prune).connections) {
		pruned_connections.insert(connection);
		if (_gg.connections.at(connection).connecting_node != -1) // remove possible connection from node in RRG
			rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.erase(
					std::remove(
							rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.begin(),
							rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.end(),
							connection),
					rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.end());
	}
}

void GlobalGraphHandler::handlePrunedFrontiers(
		const std::set<int> &pruned_targets) {
	bool removed_targets = false;
	auto pruned_target = pruned_targets.rbegin();
	while (pruned_target != pruned_targets.rend()) { // remove or add inactive targets
		deactivateFrontier(*pruned_target);
		if (*pruned_target == _gg.targets_counter - 1) {
			int next_pruned_target =
					std::next(pruned_target) == pruned_targets.rend() ?
							0 : *std::next(pruned_target); // if last pruned target, try to remove inactive targets up to index 1 (origin can't be pruned)
			for (int i = *pruned_target; i > next_pruned_target; i--) {
				if (_gg.targets.at(i).inactive) {
					_gg.targets.pop_back();
					_gg.targets_counter--;
					removed_targets = true;
				} else {
					break;
				}
			}
		} else {
			_available_targets.insert(*pruned_target);
		}
		pruned_target++;
	}
	_global_graph_searcher->rebuildIndex(_gg);
	if (removed_targets) {
		for (auto it = _available_targets.begin();
				it != _available_targets.end();) {
			if (*it >= _gg.targets_counter) {
				it = _available_targets.erase(it);
			} else {
				++it;
			}
		}
	}
}

void GlobalGraphHandler::handlePrunedPaths(const std::set<int> &pruned_connections) {
	bool removed_connections = false;
	auto pruned_connection = pruned_connections.rbegin();
	while (pruned_connection != pruned_connections.rend()) { // remove or add inactive connections
		deactivatePath(*pruned_connection);
		if (*pruned_connection == _gg.connections_counter - 1) {
			int next_pruned_connection =
					std::next(pruned_connection) == pruned_connections.rend() ?
							-1 : *std::next(pruned_connection); // if last pruned connection, try to remove inactive connections up to index 0
			for (int i = *pruned_connection; i > next_pruned_connection; i--) {
				if (_gg.connections.at(i).inactive) {
					_gg.connections.pop_back();
					_gg.connections_counter--;
					removed_connections = true;
				} else {
					break;
				}
			}
		} else {
			_available_connections.insert(*pruned_connection);
		}
		pruned_connection++;
	}
	if (removed_connections) {
		int removals = 0;
		for (auto it = _available_connections.begin(); it != _available_connections.end();
				) {
			if (*it >= _gg.connections_counter) {
				it = _available_connections.erase(it);
				removals++;
			} else {
				++it;
			}
		}
	}
}

void GlobalGraphHandler::deactivateFrontier(int pruned_target) {
	_gg.targets.at(pruned_target).inactive = true;
	_gg.targets.at(pruned_target).merged_distance = 0.0;
	_gg.targets.at(pruned_target).connections.clear();
	_gg.targets.at(pruned_target).connections_counter = 0;
	_gg.targets.at(pruned_target).viewpoint.x = 0;
	_gg.targets.at(pruned_target).viewpoint.y = 0;
}

void GlobalGraphHandler::deactivatePath(int pruned_connection) {
	if (_gg.connections.at(pruned_connection).first_target < _gg.targets_counter
			&& !_gg.targets.at(_gg.connections.at(pruned_connection).first_target).inactive) { // remove connection from list at target
		_gg.targets.at(_gg.connections.at(pruned_connection).first_target).connections.erase(
				std::remove(
						_gg.targets.at(_gg.connections.at(pruned_connection).first_target).connections.begin(),
						_gg.targets.at(_gg.connections.at(pruned_connection).first_target).connections.end(),
						pruned_connection),
				_gg.targets.at(_gg.connections.at(pruned_connection).first_target).connections.end());
		_gg.targets.at(_gg.connections.at(pruned_connection).first_target).connections_counter--;
	}
	if (_gg.connections.at(pruned_connection).second_target
			> rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH
			&& _gg.connections.at(pruned_connection).second_target < _gg.targets_counter
			&& !_gg.targets.at(_gg.connections.at(pruned_connection).second_target).inactive) { // remove connection from list at connected target
		_gg.targets.at(_gg.connections.at(pruned_connection).second_target).connections.erase(
				std::remove(
						_gg.targets.at(_gg.connections.at(pruned_connection).second_target).connections.begin(),
						_gg.targets.at(_gg.connections.at(pruned_connection).second_target).connections.end(),
						pruned_connection),
				_gg.targets.at(_gg.connections.at(pruned_connection).second_target).connections.end());
		_gg.targets.at(_gg.connections.at(pruned_connection).second_target).connections_counter--;
	}
	_gg.connections.at(pruned_connection).inactive = true;
	_gg.connections.at(pruned_connection).waypoints.clear();
	_gg.connections.at(pruned_connection).length = 0;
}

std::vector<geometry_msgs::Point> GlobalGraphHandler::pruneFrontiersAndPathsAroundNewNode(
		rrg_nbv_exploration_msgs::Graph &rrg, int new_node) {
	std::set<int> pruned_targets;
	std::set<int> pruned_connections;
	std::vector<geometry_msgs::Point> new_node_positions =
			pruneFrontiersAroundNewNode(new_node, pruned_targets,
					pruned_connections, rrg);
	if (pruned_targets.size() > 0) {
		handlePrunedFrontiers(pruned_targets);
		handlePrunedPaths(pruned_connections);
	}
	prunePathsAroundNewNode(new_node, rrg);
	return new_node_positions;
}

std::vector<geometry_msgs::Point> GlobalGraphHandler::pruneFrontiersAroundNewNode(
		int new_node, std::set<int> &pruned_targets,
		std::set<int> &pruned_connections, rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<geometry_msgs::Point> new_node_positions;
	std::vector<std::pair<int, double>> targets_near_new_node =
			_global_graph_searcher->searchInRadius(
					rrg.nodes.at(new_node).position,
					_inflation_active ?
							pow(
									sqrt(
											rrg.nodes.at(new_node).squared_radius
													- pow(_robot_width / 2, 2))
											+ _half_connection_box_distance_thres,
									2) :
							_max_edge_distance_squared); // targets in connectable distance
	for (auto target_near_new_node : targets_near_new_node) {
		if (target_near_new_node.first
				!= rrg_nbv_exploration_msgs::GlobalConnection::ORIGIN) {
			if (target_near_new_node.second
					<= (_inflation_active ?
							_robot_radius_squared : _min_edge_distance_squared)) // new node nearly at target position, prune it
					{
				tryToRewirePathsToLocalGraphOverPrunedFrontier(
						target_near_new_node.first, new_node,
						sqrt(target_near_new_node.second), rrg);
				addFrontierToBePruned(target_near_new_node.first,
						pruned_targets, pruned_connections, rrg);
			} else // try to place a node at target position to prune it
			{
				new_node_positions.push_back(
						_gg.targets.at(target_near_new_node.first).viewpoint);
			}
		}
	}
	return new_node_positions;
}

void GlobalGraphHandler::tryToRewirePathsToLocalGraphOverPrunedFrontier(
		int pruned_target, int new_node, double distance,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto connection : _gg.targets.at(pruned_target).connections) { //iterate over all target to target connections of the target to be pruned
		if (_gg.connections.at(connection).second_target
				!= rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH) { //ignore connection to local graph
			int other_target =
					pruned_target == _gg.connections.at(connection).first_target ?
							_gg.connections.at(connection).second_target :
							_gg.connections.at(connection).first_target;
			int other_targets_connection_to_local_graph = _gg.targets.at(
					other_target).connections.front();
			double new_distance = distance + _gg.connections.at(connection).length;
			if (new_distance
					< _gg.connections.at(other_targets_connection_to_local_graph).length) // new connection length through pruned target shorter than current
					{
				_gg.connections.at(other_targets_connection_to_local_graph).waypoints.clear();
				if (pruned_target == _gg.connections.at(connection).first_target) // reverse waypoints
						{
					_gg.connections.at(other_targets_connection_to_local_graph).waypoints.insert(
							_gg.connections.at(other_targets_connection_to_local_graph).waypoints.begin(),
							_gg.connections.at(connection).waypoints.rbegin(),
							_gg.connections.at(connection).waypoints.rend());
				} else {
					_gg.connections.at(other_targets_connection_to_local_graph).waypoints.insert(
							_gg.connections.at(other_targets_connection_to_local_graph).waypoints.begin(),
							_gg.connections.at(connection).waypoints.begin(),
							_gg.connections.at(connection).waypoints.end());
				}
				_gg.connections.at(other_targets_connection_to_local_graph).waypoints.push_back(
						rrg.nodes.at(new_node).position);
				_gg.connections.at(other_targets_connection_to_local_graph).length =
						new_distance;
				int connecting_node = _gg.connections.at(
						other_targets_connection_to_local_graph).connecting_node;
				rrg.nodes.at(connecting_node).connected_to.erase(
						std::remove(
								rrg.nodes.at(connecting_node).connected_to.begin(),
								rrg.nodes.at(connecting_node).connected_to.end(),
								other_targets_connection_to_local_graph),
						rrg.nodes.at(connecting_node).connected_to.end());
				_gg.connections.at(other_targets_connection_to_local_graph).connecting_node =
						new_node;
				rrg.nodes.at(new_node).connected_to.push_back(
						other_targets_connection_to_local_graph);
			}
		}
	}
}

void GlobalGraphHandler::prunePathsAroundNewNode(int new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto target : _gg.targets) { // iterate over all connections to local graph
		if (target.inactive) {
			continue;
		}
		int connection = target.connections.front(); // connection to local graph
		GlobalGraphWaypointSearcher global_connection_waypoint_searcher;
		global_connection_waypoint_searcher.rebuildIndex(_gg.connections.at(connection));
		std::vector<std::pair<int, double>> waypoints_near_new_node =
				global_connection_waypoint_searcher.searchInRadius(
						rrg.nodes.at(new_node).position,
						_inflation_active ?
								pow(
										_robot_radius
												+ rrg.nodes.at(new_node).radius,
										2) :
								_max_edge_distance_squared); // all waypoints that can be connected to the new node
		if (waypoints_near_new_node.size() > 0) {

			int closest_waypoint_to_target = getClosestWaypointToFrontier(
					connection, new_node, waypoints_near_new_node, rrg);
			if (closest_waypoint_to_target
					< (_gg.connections.at(connection).waypoints.size() - 1)) { // if connection can be pruned
				double new_connection_length = calculateNewPathLength(connection, new_node,
						closest_waypoint_to_target, rrg);
				if (new_connection_length < _gg.connections.at(connection).length) { // only re-route connection if length decreases
					rewirePathToNewNode(connection, closest_waypoint_to_target,
							new_connection_length, new_node, rrg);
					tryToImproveConnectionsToOtherFrontiers(new_node, connection,
							rrg);
					rrg.nodes.at(new_node).connected_to.push_back(connection); // add after improving connections to not filter it out during improving
				}
			}
		}
	}
}

int GlobalGraphHandler::getClosestWaypointToFrontier(int connection, int new_node,
		std::vector<std::pair<int, double>> &waypoints_near_new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	int closest_waypoint_to_target = _gg.connections.at(connection).waypoints.size() - 1; // index of last waypoint in list
// find connectable waypoint closest to the target (most connection reduction)
	if (_inflation_active) {
		for (auto waypoint_near_new_node : waypoints_near_new_node) {
			if (waypoint_near_new_node.first < closest_waypoint_to_target) { // new node must overlap with waypoint at target for re-wiring
				closest_waypoint_to_target = waypoint_near_new_node.first;
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
					_gg.connections.at(connection).waypoints.at(
							waypoint_near_new_node.first),
					sqrt(waypoint_near_new_node.second))) { // check if a connection can be made
				closest_waypoint_to_target = waypoint_near_new_node.first;
				break;
			}
		}
	}
	return closest_waypoint_to_target;
}

double GlobalGraphHandler::calculateNewPathLength(int connection, int new_node,
		int closest_waypoint_to_target,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	double new_length = 0;
	double distance_to_new_node = sqrt(
			pow(
					_gg.connections.at(connection).waypoints.at(
							closest_waypoint_to_target).x
							- rrg.nodes.at(new_node).position.x, 2)
					+ pow(
							_gg.connections.at(connection).waypoints.at(
									closest_waypoint_to_target).y
									- rrg.nodes.at(new_node).position.y, 2)); // distance between closest waypoint and new connecting node
	if ((_gg.connections.at(connection).waypoints.size() - 1 - closest_waypoint_to_target)
			> closest_waypoint_to_target) { //less waypoints remaining than pruned
		for (int i = 1; i <= closest_waypoint_to_target; i++) { // calculate distances between waypoints to remain in connection
			new_length += sqrt(
					pow(
							_gg.connections.at(connection).waypoints.at(i).x
									- _gg.connections.at(connection).waypoints.at(i - 1).x,
							2)
							+ pow(
									_gg.connections.at(connection).waypoints.at(i).y
											- _gg.connections.at(connection).waypoints.at(
													i - 1).y, 2));
		}
	} else { //less waypoints pruned than remaining
		new_length = _gg.connections.at(connection).length;
		for (int i = _gg.connections.at(connection).waypoints.size() - 1;
				i > closest_waypoint_to_target; i--) { // calculate distances between waypoints to be pruned from connection
			new_length -= sqrt(
					pow(
							_gg.connections.at(connection).waypoints.at(i).x
									- _gg.connections.at(connection).waypoints.at(i - 1).x,
							2)
							+ pow(
									_gg.connections.at(connection).waypoints.at(i).y
											- _gg.connections.at(connection).waypoints.at(
													i - 1).y, 2));
		}
	}
	return (new_length + distance_to_new_node);
}

void GlobalGraphHandler::rewirePathToNewNode(int connection,
		int closest_waypoint_to_target, double new_connection_length, int new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::string cons = "";
	for (auto con_connection : rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to) {
		cons += std::to_string(con_connection) + ",";
	}
	rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.erase(
			std::remove(
					rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.begin(),
					rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.end(),
					connection),
			rrg.nodes.at(_gg.connections.at(connection).connecting_node).connected_to.end()); // remove connection from connecting node in RRG
	_gg.connections.at(connection).waypoints.erase(
			_gg.connections.at(connection).waypoints.begin() + closest_waypoint_to_target
					+ 1, _gg.connections.at(connection).waypoints.end());
	_gg.connections.at(connection).waypoints.push_back(rrg.nodes.at(new_node).position);
	_gg.connections.at(connection).length = new_connection_length;
	_gg.connections.at(connection).connecting_node = new_node;
}

void GlobalGraphHandler::tryToImproveConnectionsToOtherFrontiers(int new_node,
		int connection, rrg_nbv_exploration_msgs::Graph &rrg) {
	int current_target = _gg.connections.at(connection).first_target; // look for new connections with this target
	std::set<int> new_target_connections; // set of existing connected targets to the connecting node which will be reduced by the targets already connected to the current target
	std::vector<std::pair<int, double>> mergeable_targets; // target index first and distance to local graph second
	std::pair<int, double> closest_mergeable_unprunable_target( { -1,
			std::numeric_limits<double>::infinity() }); // target index first and distance to local graph second
	for (auto connected_connection : rrg.nodes.at(new_node).connected_to) {
		new_target_connections.insert(_gg.connections.at(connected_connection).first_target);
	}
	for (auto target_connection : _gg.targets.at(current_target).connections) {
		int other_target =
				current_target == _gg.connections.at(target_connection).first_target ?
						_gg.connections.at(target_connection).second_target :
						_gg.connections.at(target_connection).first_target;
		if (_gg.connections.at(target_connection).second_target
				!= rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH
				&& new_target_connections.erase(other_target) > 0) { // remove existing connection from new target connections (if present, try to improve connection)
			int other_target_connection_to_local_graph = _gg.targets.at(
					other_target).connections.front();
			if ((_gg.connections.at(connection).length
					+ _gg.connections.at(other_target_connection_to_local_graph).length)
					< _gg.connections.at(target_connection).length) { // connection through new node would be shorter
				improvePathToConnectedFrontier(target_connection, connection,
						other_target, other_target_connection_to_local_graph);

			}
		}
	}
	for (auto new_target_connection : new_target_connections) {
		connectFrontiers(current_target, new_target_connection, connection,
				_gg.targets.at(new_target_connection).connections.front(), false); // first connection of target is always connection to local graph
	}
}

void GlobalGraphHandler::improvePathToConnectedFrontier(int target_connection,
		int connection, int other_target, int other_connection) {
	_gg.connections.at(target_connection).waypoints.clear();
	_gg.connections.at(target_connection).length = _gg.connections.at(connection).length
			+ _gg.connections.at(_gg.targets.at(other_target).connections.at(0)).length;
	_gg.connections.at(target_connection).waypoints.insert(
			_gg.connections.at(target_connection).waypoints.begin(),
			_gg.connections.at(connection).waypoints.begin(),
			_gg.connections.at(connection).waypoints.end() - 1); // omit last element to avoid duplicate waypoint
	_gg.connections.at(target_connection).waypoints.insert(
			_gg.connections.at(target_connection).waypoints.end(),
			_gg.connections.at(other_connection).waypoints.rbegin(),
			_gg.connections.at(other_connection).waypoints.rend());
}

bool GlobalGraphHandler::calculateNextFrontierGoal(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<int> active_targets;
	for (auto &target : _gg.targets) {
		if (!target.inactive)
			active_targets.push_back(target.index);
	}
	if (active_targets.size() == 1 && _auto_homing) { // only origin remains
		ROS_INFO_STREAM("Next target goal is the origin, going home");
		connectPathsToLocalGraphToNearestNode(rrg);
		_global_route.push_back(std::make_pair(0, 0));
		_next_global_goal = 0;
	} else if (active_targets.size() <= 1) {
		ROS_INFO_STREAM("No more target goals available");
		return false;
	} else {
		establishMissingFrontierToFrontierConnections(active_targets, rrg);
		connectPathsToLocalGraphToNearestNode(rrg);
		std::pair<int, int> next_target_with_connection =
				findBestFrontierWithTspTwoOpt(active_targets);
		if (next_target_with_connection.first == -1
				|| next_target_with_connection.second == -1) {
			ROS_WARN_STREAM(
					"Unable to determine the next target goal with 2-Opt");
			return false;
		}
		_next_global_goal = 1;
		ROS_INFO_STREAM(
				"Next target goal is " << _global_route.at(_next_global_goal).first << " with connection " << _global_route.at(_next_global_goal).second);
	}
	ROS_INFO_STREAM(
			"Rebuild global connection waypoint searcher for connection " << _global_route.at(_next_global_goal).second);
	_global_connection_waypoint_searcher->rebuildIndex(
			_gg.connections.at(_global_route.at(_next_global_goal).second));
	ROS_INFO_STREAM(
			"Set closest waypoint to " << _gg.connections.at(_global_route.at(_next_global_goal).second).waypoints.size() - 1);
	_active_connections_closest_waypoint =
			std::make_pair(_global_route.at(_next_global_goal).second,
					_gg.connections.at(_global_route.at(_next_global_goal).second).waypoints.size()
							- 1); // start at waypoint at nearest node in RRG
	return true;
}

void GlobalGraphHandler::debugRoute(std::vector<std::pair<int, int> > &route,
		std::string prefix, double length) {
	std::string s = prefix + " route: ";
	for (auto n : route) {
		s += "(" + std::to_string(n.first) + ")-" + std::to_string(n.second)
				+ "->";
	}
	s += " dist: " + std::to_string(length);
	ROS_INFO_STREAM(s);
}

bool GlobalGraphHandler::sortByPathLengthToFrontier(int target_one,
		int target_two) {
	return _gg.connections.at(_gg.targets.at(target_one).connections.front()).length
			< _gg.connections.at(_gg.targets.at(target_two).connections.front()).length;
}

std::pair<int, int> GlobalGraphHandler::findBestFrontierWithTspTwoOpt(
		std::vector<int> &active_targets) {
	std::vector<std::pair<int, int>> route; // target index (first) and connection index from previous target in route (second)
	auto origin = std::find(active_targets.begin(), active_targets.end(),
			rrg_nbv_exploration_msgs::GlobalConnection::ORIGIN);
	if (origin != active_targets.end()) { // remove origin from active targets (will be pushed back later when homing)
		active_targets.erase(origin);
	}

	// sort active targets ascending by global connection to target length
	std::sort(active_targets.begin(), active_targets.end(),
			[this](int target_one, int target_two) {
				return sortByPathLengthToFrontier(target_one, target_two);
			});
	if (_auto_homing) {
		active_targets.push_back(
				rrg_nbv_exploration_msgs::GlobalConnection::ORIGIN); // origin as last target
	}
	route.push_back(
			std::make_pair(rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH,
					-1)); // start at robot position (local graph) with no connection
	route.push_back(
			std::make_pair(active_targets.front(),
					_gg.targets.at(active_targets.front()).connections.front())); // add first active target and connection to local graph to route

	for (int i = 1; i < active_targets.size(); i++) { // add target index and index of connection from previous target to route except for first entry
		int connection_index = findPathToNextFrontier(active_targets.at(i - 1),
				active_targets.at(i));
		if (connection_index == -1) {
			return std::make_pair(-1, -1);
		}
		route.push_back(std::make_pair(active_targets.at(i), connection_index));
	}
//	debugRoute(route, "initial", calculateRouteLength(route));
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
	for (int i = 1; i < route.size() - 1; i++) { // omit first target (robot position/local graph)
		for (int k = i + 1; k < route.size() - (_auto_homing ? 1 : 0); k++) { // omit last target (origin) when auto homing
			std::vector<std::pair<int, int>> new_route;
			if (twoOptSwap(route, new_route, i, k)) { // if swap resulted in traversable route
				double new_distance = calculateRouteLength(new_route);
				if (new_distance < best_distance) {
//					debugRoute(new_route, "superior swap", new_distance);
					route = new_route;
					best_distance = new_distance;
					return true;
				}
//				else {
//					debugRoute(new_route, "inferior swap", new_distance);
//				}
			}
		}
	}
	return false;
}

int GlobalGraphHandler::findPathToNextFrontier(int current_target,
		int next_target) {
	int connection_index = -1;
	if (current_target == rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH)
		connection_index = _gg.targets.at(next_target).connections.front();
	else if (next_target == rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH)
		connection_index = _gg.targets.at(current_target).connections.front();
	else {
		for (auto connection : _gg.targets.at(current_target).connections) {
			if (!_gg.connections.at(connection).inactive
					&& (_gg.connections.at(connection).first_target == next_target
							|| _gg.connections.at(connection).second_target == next_target)) {
				connection_index = connection;
				break;
			}
		}
	}
	return connection_index;
}

double GlobalGraphHandler::calculateRouteLength(
		std::vector<std::pair<int, int>> &route) {
	double distance = 0.0;
	for (int i = 1; i < route.size(); i++) { // omit first target (local graph) in route
		distance += _gg.connections.at(route.at(i).second).length;
	}
	return distance;
}

bool GlobalGraphHandler::twoOptSwap(std::vector<std::pair<int, int>> &route,
		std::vector<std::pair<int, int>> &new_route, int i, int k) {
	for (int j = 0; j < i; j++) {
		new_route.push_back(route.at(j));
	}
	for (int l = k; l >= i; l--) {
		int connection_index = findPathToNextFrontier(new_route.back().first,
				route.at(l).first);
		new_route.push_back(route.at(l));
		new_route.back().second = connection_index;
	}
	for (int m = k + 1; m < route.size(); m++) {
		new_route.push_back(route.at(m));
	}
	return true;
}

void GlobalGraphHandler::establishMissingFrontierToFrontierConnections(
		std::vector<int> active_targets,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<ShortestFrontierConnectionStruct> target_connections;
	for (auto &target : _gg.targets) { // establish all missing connection between targets through the RRG
		if (!target.inactive
				&& target.connections_counter < active_targets.size()) { // one connection to every other target + 1 to RRG required
			std::set<int> missing_targets = getMissingFrontierConnections(
					active_targets, target);
			int target_connecting_node =
					_gg.connections.at(target.connections.front()).connecting_node;
			double max_distance_threshold = 0; // maximum connection length threshold when searching for connections
			std::vector<std::pair<int, int>> missing_target_connections; // missing target index (first) and connecting node of missing target to local graph (second)
			for (auto missing_target : missing_targets) { // build the missing connections
				if (target.index > missing_target) { // only build connections where this target will be target in connection
					int missing_target_connecting_node =
							_gg.connections.at(
									_gg.targets.at(missing_target).connections.front()).connecting_node;
					bool found_existing_connection = false;
					for (auto target_connection : target_connections) { //check if a connection between the connecting nodes of both targets was already found and reuse it
						if (target_connection.connecting_node_one
								== std::max(target_connecting_node,
										missing_target_connecting_node)
								&& target_connection.connecting_node_two
										== std::min(target_connecting_node,
												missing_target_connecting_node)) {
							found_existing_connection = true;
							buildMissingPathBetweenFrontiers(target,
									missing_target,
									target_connection.path_length,
									target_connection.path, rrg);
							break;
						}
					}
					if (!found_existing_connection) {
						missing_target_connections.push_back(
								std::make_pair(missing_target,
										missing_target_connecting_node));
						findShortestPathThroughMutualNode(
								target_connecting_node,
								missing_target_connecting_node,
								max_distance_threshold, rrg);
					}
				}
			}
			if (!missing_target_connections.empty()) { // only find shortest routes if a connection is missing
				std::vector<ShortestFrontierConnectionStruct> local_connections;
				std::map<int, int> missing_target_local_connection_map =
						_graph_connection_calculator->findShortestRoutes(rrg,
								target_connecting_node,
								missing_target_connections, local_connections,
								max_distance_threshold);
				target_connections.insert(target_connections.end(),
						local_connections.begin(), local_connections.end()); // add local connections to target connections
				for (auto missing_target_local_connection : missing_target_local_connection_map) { // iterate over map (first=missing target index, second=local connection index)
					buildMissingPathBetweenFrontiers(target,
							missing_target_local_connection.first,
							local_connections.at(missing_target_local_connection.second).path_length,
							local_connections.at(missing_target_local_connection.second).path,
							rrg);
				}
			}
		}
	}
}

std::set<int> GlobalGraphHandler::getMissingFrontierConnections(
		std::vector<int> &active_targets,
		rrg_nbv_exploration_msgs::GlobalTarget &target) {
	std::set<int> missing_targets(active_targets.begin(),
			active_targets.end()); // one connection to every other target + 1 to RRG required
	missing_targets.erase(target.index);
	for (int connection : target.connections) {
		if (_gg.connections.at(connection).second_target
				!= rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH)
			missing_targets.erase(
					target.index == _gg.connections.at(connection).first_target ?
							_gg.connections.at(connection).second_target :
							_gg.connections.at(connection).first_target);
	}
	return missing_targets;
}

void GlobalGraphHandler::findShortestPathThroughMutualNode(
		int target_connecting_node, int missing_target_connecting_node,
		double &max_distance_threshold, rrg_nbv_exploration_msgs::Graph &rrg) {
	int first_mutual_node_in_connection = rrg.nearest_node;
	double minimum_local_connection_length =
			rrg.nodes.at(target_connecting_node).distance_to_robot
					+ rrg.nodes.at(missing_target_connecting_node).distance_to_robot;
	std::vector<int> mutual_node_local_connection; // the shortest possible connection between the two targets using a mutual node in both connecting node's connections to robot
	for (auto rit =
			rrg.nodes.at(target_connecting_node).path_to_robot.rbegin();
			rit
					!= std::prev(
							rrg.nodes.at(target_connecting_node).path_to_robot.rend());
			++rit) { // reverse end is connecting node
		auto it =
				std::find(
						std::next(
								rrg.nodes.at(missing_target_connecting_node).path_to_robot.begin()), // first node is connecting node
						rrg.nodes.at(missing_target_connecting_node).path_to_robot.end(),
						*rit); // find occurence of node in both connections
		if (it
				!= rrg.nodes.at(missing_target_connecting_node).path_to_robot.end()) { // found mutual node in connections
			first_mutual_node_in_connection = *it;
			mutual_node_local_connection.insert(mutual_node_local_connection.end(),
					rrg.nodes.at(target_connecting_node).path_to_robot.rbegin(),
					rit);
			mutual_node_local_connection.insert(mutual_node_local_connection.end(), it,
					rrg.nodes.at(missing_target_connecting_node).path_to_robot.end());
			std::string rempa = "";
			for (int rem : mutual_node_local_connection) {
				rempa += std::to_string(rem) + ",";
			}
			double length = 0;
			for (int i = 0; i < mutual_node_local_connection.size() - 1; i++) { // iterate over edges in remaining connection to calculate length
				int edge = _graph_connection_calculator->findExistingEdge(rrg,
						mutual_node_local_connection.at(i),
						mutual_node_local_connection.at(i + 1));
				if (edge != -1) {
					length += rrg.edges.at(edge).length;
				} else {
					ROS_WARN_STREAM(
							"Unable to calculate minimum local connection, node " << mutual_node_local_connection.at(i) << " has no edge to " << mutual_node_local_connection.at(i + 1));
					length = 0;
					break;
				}
			}
			if (length > 0) {
				minimum_local_connection_length = length;
				break;
			}
		}
	}
	if (mutual_node_local_connection.empty()) {
		max_distance_threshold = std::numeric_limits<double>::infinity(); // cannot use distance threshold
	}
	max_distance_threshold = std::max(max_distance_threshold,
			minimum_local_connection_length);
}

void GlobalGraphHandler::buildMissingPathBetweenFrontiers(
		rrg_nbv_exploration_msgs::GlobalTarget_<std::allocator<void>> &target,
		int missing_target, double connection_length, std::vector<int> &local_connection,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	rrg_nbv_exploration_msgs::GlobalConnection connection_between_targets;
	connection_between_targets.index = availablePathIndex();
	connection_between_targets.first_target = target.index;
	connection_between_targets.second_target = missing_target;
	connection_between_targets.waypoints.insert(
			connection_between_targets.waypoints.begin(),
			_gg.connections.at(_gg.targets.at(target.index).connections.front()).waypoints.begin(),
			_gg.connections.at(_gg.targets.at(target.index).connections.front()).waypoints.end());
	for (int i = 1; i < local_connection.size() - 1; i++) { // omit begin and end, connecting nodes are already present in connection
		connection_between_targets.waypoints.push_back(
				rrg.nodes.at(local_connection.at(i)).position);
	}
	connection_between_targets.waypoints.insert(
			connection_between_targets.waypoints.end(),
			_gg.connections.at(_gg.targets.at(missing_target).connections.front()).waypoints.rbegin(),
			_gg.connections.at(_gg.targets.at(missing_target).connections.front()).waypoints.rend());
	connection_between_targets.length =
			_gg.connections.at(_gg.targets.at(target.index).connections.front()).length
					+ connection_length
					+ _gg.connections.at(
							_gg.targets.at(missing_target).connections.front()).length;
	connection_between_targets.connecting_node = -1;
	_gg.targets.at(target.index).connections.push_back(
			connection_between_targets.index);
	_gg.targets.at(target.index).connections_counter++;
	_gg.targets.at(missing_target).connections.push_back(
			connection_between_targets.index);
	_gg.targets.at(missing_target).connections_counter++;
	insertPathInGg(connection_between_targets);
}

void GlobalGraphHandler::connectPathsToLocalGraphToNearestNode(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto &connection : _gg.connections) { // connect all targets directly to the nearest node to the robot
		if (!connection.inactive
				&& connection.second_target
						== rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH) { // add connection to robot from connecting node
			for (int i = rrg.nodes.at(connection.connecting_node).path_to_robot.size()
					- 2; i >= 0; i--) { // skip node itself, as it already is in waypoints
				connection.waypoints.push_back(
						rrg.nodes.at(
								rrg.nodes.at(connection.connecting_node).path_to_robot.at(
										i)).position);
			}
			connection.length += rrg.nodes.at(connection.connecting_node).distance_to_robot;
			connection.connecting_node = rrg.nearest_node;
		}
	}
}

bool GlobalGraphHandler::checkIfNextFrontierWithPathIsValid() {
	return _next_global_goal >= 0 && _next_global_goal < _global_route.size()
			&& _global_route.at(_next_global_goal).first >= 0
			&& _global_route.at(_next_global_goal).first < _gg.targets_counter
			&& _global_route.at(_next_global_goal).second >= 0
			&& _global_route.at(_next_global_goal).second < _gg.connections_counter;
}

bool GlobalGraphHandler::getFrontierGoal(geometry_msgs::Point &goal,
		double &yaw, geometry_msgs::Point &robot_pos) {
	if (checkIfNextFrontierWithPathIsValid()) {
		goal =
				_gg.targets.at(_global_route.at(_next_global_goal).first).viewpoint;
		if (_gg.connections.at(_global_route.at(_next_global_goal).second).waypoints.size()
				> 1) {
			yaw =
					(int) (atan2(
							goal.y
									- _gg.connections.at(
											_global_route.at(_next_global_goal).second).waypoints.at(
											1).y,
							goal.x
									- _gg.connections.at(
											_global_route.at(_next_global_goal).second).waypoints.at(
											1).x) * 180 / M_PI);
		} else { // target directly next to robot, define yaw from robot position
			yaw = (int) (atan2(goal.y - robot_pos.y, goal.x - robot_pos.x) * 180
					/ M_PI);
		}
		return true;
	} else {
		return false;
	}
}

bool GlobalGraphHandler::getFrontierPath(
		std::vector<geometry_msgs::PoseStamped> &connection,
		geometry_msgs::Point &robot_pos) {
	if (checkIfNextFrontierWithPathIsValid()) {
		ROS_INFO_STREAM(
				"Get connection to target " << _global_route.at(_next_global_goal).first);
		std::vector<geometry_msgs::Point> waypoints;
		if (_previous_global_goal_failed >= 0) {
			// Add connection from local graph to next target but skip last waypoint as it is always present in the connection
			ROS_INFO_STREAM(
					"Start with connection to " << _global_route.at(_next_global_goal).first);
			waypoints.insert(waypoints.end(),
					_gg.connections.at(
							_gg.targets.at(
									_global_route.at(_next_global_goal).first).connections.front()).waypoints.begin(),
					std::prev(
							_gg.connections.at(
									_gg.targets.at(
											_global_route.at(_next_global_goal).first).connections.front()).waypoints.end()));
			// go back to former local graph to start navigation to next target from there
			ROS_INFO_STREAM(
					"Add reversed connection from " << _previous_global_goal_failed);
			waypoints.insert(waypoints.end(),
					_gg.connections.at(
							_gg.targets.at(_previous_global_goal_failed).connections.front()).waypoints.rbegin(),
					_gg.connections.at(
							_gg.targets.at(_previous_global_goal_failed).connections.front()).waypoints.rend());
			// prune waypoints from the connection back to the local graph up to the nearest one to the robot
			rrg_nbv_exploration_msgs::GlobalConnection connection_to_next_goal_from_failed_goal;
			connection_to_next_goal_from_failed_goal.waypoints = waypoints;
			//rebuild waypoint searcher to update closest waypoint
			_global_connection_waypoint_searcher->rebuildIndex(
					connection_to_next_goal_from_failed_goal);
			double min_distance;
			int nearest_waypoint_index = -1;
			_global_connection_waypoint_searcher->findNearestNeighbour(robot_pos,
					min_distance, nearest_waypoint_index);
//			ROS_INFO_STREAM(connection_to_next_goal_from_failed_goal);
			ROS_INFO_STREAM(
					sqrt(min_distance) << "m to nearest waypoint  " << nearest_waypoint_index << " at (" << waypoints.at(nearest_waypoint_index).x << ","<< waypoints.at(nearest_waypoint_index).y << ")");
			//update closest waypoint
			_active_connections_closest_waypoint = std::make_pair(
					_global_route.at(_next_global_goal).second,
					nearest_waypoint_index); // start at waypoint at nearest node in RRG
		} else {
			waypoints.insert(waypoints.end(),
					_gg.connections.at(_global_route.at(_next_global_goal).second).waypoints.begin(),
					_gg.connections.at(_global_route.at(_next_global_goal).second).waypoints.end());
			ROS_INFO_STREAM(
					"Inserted waypoints of connection " << _global_route.at(_next_global_goal).second);
		}
		ROS_INFO_STREAM(
				"Waypoints: " << waypoints.size() << ", closest wp: " << _active_connections_closest_waypoint.second);
		if (waypoints.size() == 0) {
			ROS_INFO_STREAM("Waypoints empty, insert target waypoint");
			waypoints.push_back(
					_gg.targets.at(_global_route.at(_next_global_goal).first).viewpoint);
		}
		_graph_connection_calculator->getNavigationPath(connection, waypoints, robot_pos,
				_active_connections_closest_waypoint.second);
		return true;
	} else {
		return false;
	}
}

std::vector<int> GlobalGraphHandler::targetReached(
		geometry_msgs::Point &position) {
	std::vector<int> connected_connections;
	if (checkIfNextFrontierWithPathIsValid()) {
		int target = _global_route.at(_next_global_goal).first;
		if(_auto_homing && target == 0) { //reached home
			return connected_connections;
		}
		position = _gg.targets.at(target).viewpoint;
		_next_global_goal = -1;
		_global_route.clear();
		_previous_global_goal_failed = -1;
		std::set<int> pruned_targets;
		std::set<int> pruned_connections;
		pruned_targets.insert(target);
		for (auto connection : _gg.targets.at(target).connections) {
			pruned_connections.insert(connection);
			overwritePathToLocalGraph(connection, target, connected_connections);
		}
		if (pruned_targets.size() > 0) {
			handlePrunedFrontiers(pruned_targets);
			handlePrunedPaths(pruned_connections);
		}
	}
	return connected_connections;
}

bool GlobalGraphHandler::targetFailed() {
	if (_next_global_goal >= _global_route.size() - 1) { // last target in route, exploration finished
		return true;
	} else {
		_previous_global_goal_failed =
				_global_route.at(_next_global_goal).first;
		_next_global_goal++;
		ROS_INFO_STREAM(
				"Frontier " << _global_route.at(_next_global_goal - 1).first << " failed, try next target " << _global_route.at(_next_global_goal).first);
		return false;
	}
}

void GlobalGraphHandler::overwritePathToLocalGraph(int connection, int target,
		std::vector<int> &connected_connections) {
	if (_gg.connections.at(connection).second_target
			!= rrg_nbv_exploration_msgs::GlobalConnection::LOCAL_GRAPH) {
		int other_target =
				_gg.connections.at(connection).first_target == target ?
						_gg.connections.at(connection).second_target :
						_gg.connections.at(connection).first_target;
		int other_targets_connection_to_local_graph = _gg.targets.at(
				other_target).connections.front();
		// overwrite connection between other target and local graph with details from the connection between those two
		_gg.connections.at(other_targets_connection_to_local_graph).connecting_node = 0; // root at new RRG
		_gg.connections.at(other_targets_connection_to_local_graph).waypoints.clear();
		if (other_target > target) {
			_gg.connections.at(other_targets_connection_to_local_graph).waypoints.insert(
					_gg.connections.at(other_targets_connection_to_local_graph).waypoints.end(),
					_gg.connections.at(connection).waypoints.begin(),
					_gg.connections.at(connection).waypoints.end());
		} else { // connection was from target to other target, must be reversed
			_gg.connections.at(other_targets_connection_to_local_graph).waypoints.insert(
					_gg.connections.at(other_targets_connection_to_local_graph).waypoints.end(),
					_gg.connections.at(connection).waypoints.rbegin(),
					_gg.connections.at(connection).waypoints.rend());
		}
		_gg.connections.at(other_targets_connection_to_local_graph).length = _gg.connections.at(
				connection).length;
		connected_connections.push_back(other_targets_connection_to_local_graph);
	}
}

bool GlobalGraphHandler::updateClosestWaypoint(
		geometry_msgs::Point &robot_pos) {
	double min_distance;
	int nearest_node;
	_global_connection_waypoint_searcher->findNearestNeighbour(robot_pos,
			min_distance, nearest_node);
	if (nearest_node == 0) { // target reached
		ROS_INFO_STREAM("Frontier waypoint reached " << nearest_node);
		_active_connections_closest_waypoint = std::make_pair(-1, -1);
		return true;
	}
	if (nearest_node != _active_connections_closest_waypoint.second) {
		ROS_INFO_STREAM("New closest waypoint " << nearest_node);
		_active_connections_closest_waypoint.second = nearest_node;
	}
	return false;
}

void GlobalGraphHandler::dynamicReconfigureCallback(
		rrg_nbv_exploration::GraphConstructorConfig &config, uint32_t level) {
	_local_graph_radius = std::max(config.local_graph_radius,
			2 * _robot_radius);
}

} /* namespace rrg_nbv_exploration */
