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
	_local_graph_radius_squared = pow(_local_graph_radius, 2);

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
	_gg.header.frame_id = "/map";
	_gg.ns = "globalgraph";
	_gg.frontiers_counter = 0;
	_gg.paths_counter = 0;
	rrg_nbv_exploration_msgs::GlobalGraph test = _gg;
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
	// TODO: debug path with negative length
	for (auto path : _gg.paths) {
		if (path.length < 0) {
			ROS_ERROR_STREAM(
					"Path " << path.index << " length: " << path.length);
		}
	}
	for (auto frontier : _gg.frontiers) {
		if (frontier.merged_distance > _local_graph_radius) {
			ROS_ERROR_STREAM(
					"Frontier " << frontier.index << " merged distance: " << frontier.merged_distance);
		}
	}
	_global_graph_publisher.publish(_gg);
}

bool GlobalGraphHandler::getConnectingNode(int node,
		rrg_nbv_exploration_msgs::Graph &rrg,
		std::vector<geometry_msgs::Point> &waypoints, int &connecting_node,
		double &length) {
//	ROS_INFO_STREAM("Get connecting node for node " << node);
	if (rrg.nodes.at(node).path_to_robot.size() > 1) {
		int next_node = rrg.nodes.at(node).path_to_robot.size() - 2;
		int edge = _graph_path_calculator->findExistingEdge(rrg,
				rrg.nodes.at(node).path_to_robot.at(next_node + 1),
				rrg.nodes.at(node).path_to_robot.at(next_node));
		//		ROS_INFO_STREAM(
		//				"Edge to connecting node " << rrg.nodes.at(node).path_to_robot.at(next_node)<< " is " << edge);
		if (edge >= rrg.edges.size()) {
			ROS_ERROR_STREAM(
					"Node " << node << " links to an edge " << edge << "to node " << rrg.nodes.at(node).path_to_robot.at(next_node) << " that was already removed!");
			return false;
		}
		if (edge != -1) {
			connecting_node = rrg.nodes.at(node).path_to_robot.at(next_node);
			length += rrg.edges.at(edge).length;
			waypoints.push_back(
					rrg.nodes.at(rrg.nodes.at(node).path_to_robot.at(next_node)).position);
			ROS_INFO_STREAM(
					"New connecting node " << rrg.nodes.at(node).path_to_robot.at(next_node) << " with edge length " << length << " to former node " << node);
			return true;
		}
	}
	return false;
}

void GlobalGraphHandler::insertFrontierInGg(
		const rrg_nbv_exploration_msgs::GlobalFrontier &frontier) {
	if (!_available_frontiers.empty()) {
		ROS_INFO_STREAM("Insert frontier at " << *_available_frontiers.begin());
		_gg.frontiers.at(*_available_frontiers.begin()) = frontier;
		_available_frontiers.erase(_available_frontiers.begin());
	} else {
		ROS_INFO_STREAM("Push frontier at " << _gg.frontiers_counter);
		_gg.frontiers.push_back(frontier);
		_gg.frontiers_counter++;
	}
}

void GlobalGraphHandler::insertPathInGg(
		const rrg_nbv_exploration_msgs::GlobalPath &path_between_frontiers) {
	if (!_available_paths.empty()) {
		ROS_INFO_STREAM("Insert path at " << *_available_paths.begin());
		_gg.paths.at(*_available_paths.begin()) = path_between_frontiers;
		_available_paths.erase(_available_paths.begin());
	} else {
		ROS_INFO_STREAM("Push path at " << _gg.paths_counter);
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
	ROS_INFO_STREAM(
			"+++++ Add frontier " << frontier.index << " at node " << node);
	if (!getConnectingNode(node, rrg, path.waypoints, path.connecting_node,
			path.length)) {
		ROS_WARN_STREAM(
				"Unable to add frontier for node " << node << ", found no connection to the RRG");
		return;
	}
	//TODO: remove, only for debugging
	path.index = availablePathIndex();
	//end remove
	if (tryToMergeAddedFrontiers(node, path, rrg, frontier)) { // if new frontier is merged into existing
		return;
	}
	path.index = availablePathIndex();
	path.frontier = frontier.index;
	frontier.paths.push_back(path.index);
	frontier.paths_counter++;
	insertFrontierInGg(frontier);
	insertPathInGg(path);
	ROS_INFO_STREAM(
			"Add path " << path.index << " to frontier " << frontier.index << " with length " << path.length << " and connecting node " << path.connecting_node);
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
	ROS_INFO_STREAM(
			"----- Added frontier " << frontier.index << " at node " << node);
}

void GlobalGraphHandler::continuePath(int node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<geometry_msgs::Point> additional_waypoints;
	ROS_INFO_STREAM("+++++ Continue path at node " << node);
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
			ROS_INFO_STREAM(
					"Path " << path << " to frontier " << _gg.paths.at(path).frontier <<" that is connected to " << node << " was reconnected to node " << connecting_node << " and has new length " << _gg.paths.at(path).length << " (+ " <<length << ")");
			rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.push_back(
					path);
			if (connecting_node_frontiers.size() > 0) {
				int current_frontier = _gg.paths.at(path).frontier; // look for new connections with this frontier
				std::set<int> new_frontier_connections;	// set of existing connected frontiers to the connecting node which will be reduced by the frontiers already connected to the current frontier
				for (auto elem : connecting_node_frontiers) {
					new_frontier_connections.insert(elem.first);
				}
				for (auto frontier_path : _gg.frontiers.at(current_frontier).paths) {
//				ROS_INFO_STREAM(
//						"Look at path " << frontier_path << " at current frontier " << current_frontier);
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
	ROS_INFO_STREAM("----- Continued path at node " << node);
}

bool GlobalGraphHandler::tryToMergeContinuedPaths(int current_frontier,
		int path, std::map<int, int> &connecting_node_frontiers,
		std::set<int> &new_frontier_connections,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM(
			"+++++ tryToMergeContinuedPaths for current frontier " << current_frontier);
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
		ROS_INFO_STREAM(
				"Frontier " << current_frontier << " was merged into existing frontier " << closest_mergeable_unprunable_frontier.frontier << " with merged distance " << (_gg.frontiers.at(closest_mergeable_unprunable_frontier.frontier).merged_distance));
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
		ROS_INFO_STREAM(
				"Remaining frontier " << remaining_frontier << " with length " << shortest_path_length);
		for (auto mergeable_frontier : mergeable_frontiers) { // find remaining frontier
			merged_distances_to_frontiers.insert(
					std::make_pair(
							_gg.frontiers.at(mergeable_frontier.frontier).merged_distance
									+ mergeable_frontier.distance_between_frontiers,
							mergeable_frontier.frontier));
			if (mergeable_frontier.path_length_to_local_graph
					<= shortest_path_length) {
				if (remaining_frontier != current_frontier) { // not in graph, cannot be pruned
					ROS_INFO_STREAM(
							"Remaining frontier " << remaining_frontier << " with length " << shortest_path_length << " pruned");
					addFrontierToBePruned(remaining_frontier, pruned_frontiers,
							pruned_paths, rrg);
					removeFrontierFromConnectableFrontierList(
							remaining_frontier, connecting_node_frontiers,
							new_frontier_connections);
				}
				ROS_INFO_STREAM(
						"Remaining frontier " << remaining_frontier << " replaced with frontier " << mergeable_frontier.frontier << " with length " << mergeable_frontier.path_length_to_local_graph << " with merged distance: " << (_gg.frontiers.at(mergeable_frontier.frontier).merged_distance + mergeable_frontier.distance_between_frontiers));
				remaining_frontier = mergeable_frontier.frontier;
				shortest_path_length =
						mergeable_frontier.path_length_to_local_graph;
				distance_current_frontier_to_remaining_frontier =
						mergeable_frontier.distance_between_frontiers;
			} else {
				ROS_INFO_STREAM(
						"Mergeable frontier " << mergeable_frontier.frontier << " with length " << mergeable_frontier.path_length_to_local_graph << " pruned, with merged distance: " << (_gg.frontiers.at(mergeable_frontier.frontier).merged_distance + mergeable_frontier.distance_between_frontiers));
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
				ROS_INFO_STREAM(
						"Remaining frontier " << remaining_frontier << " pruned because of max merged distance violation, its merged distance is " << max_relevant_merged_distance);
				addFrontierToBePruned(remaining_frontier, pruned_frontiers,
						pruned_paths, rrg);
				removeFrontierFromConnectableFrontierList(remaining_frontier,
						connecting_node_frontiers, new_frontier_connections);
				_gg.frontiers.at(current_frontier).merged_distance =
						merged_distances_to_frontiers.rbegin()->first;
				ROS_INFO_STREAM(
						"Therefore current frontier " << current_frontier << " remains after merge with merged distance " << merged_distances_to_frontiers.rbegin()->first;);
			} else { //merge like all other frontiers were merged into current frontier and then into remaining frontier
				addFrontierToBePruned(current_frontier, pruned_frontiers,
						pruned_paths, rrg);
				_gg.frontiers.at(remaining_frontier).merged_distance = std::max(
						max_relevant_merged_distance,
						_gg.frontiers.at(remaining_frontier).merged_distance);
				ROS_INFO_STREAM(
						"Frontier " << current_frontier << " was merged into existing frontier " << remaining_frontier << " with merged distance " << _gg.frontiers.at(remaining_frontier).merged_distance);
				pruned_current_frontier = true;
			}
		} else {
			_gg.frontiers.at(current_frontier).merged_distance =
					merged_distances_to_frontiers.rbegin()->first;
			ROS_INFO_STREAM(
					"Current frontier " << current_frontier << " remains after merge with merged distance " << merged_distances_to_frontiers.rbegin()->first;);
		}
	}
	if (pruned_frontiers.size() > 0) {
		handlePrunedFrontiers(pruned_frontiers);
		handlePrunedPaths(pruned_paths);
	}
	ROS_INFO_STREAM("----- tryToMergeContinuedPaths");
	return pruned_current_frontier;
}

void GlobalGraphHandler::removeFrontierFromConnectableFrontierList(int frontier,
		std::map<int, int> &connecting_node_frontiers,
		std::set<int> &new_frontier_connections) {
	ROS_INFO_STREAM(
			"removeFrontierFromConnectableFrontierList frontier " << frontier);
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
		ROS_INFO_STREAM(
				"tryToMergeAddedFrontiers for frontier " << frontier.index);
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
			ROS_INFO_STREAM(
					"New frontier " << frontier.index << " was merged into existing frontier " << closest_mergeable_unprunable_frontier.frontier<< " with merged distance " << (_gg.frontiers.at(closest_mergeable_unprunable_frontier.frontier).merged_distance));
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
						ROS_INFO_STREAM(
								"Remaining frontier " << remaining_frontier << " with length " << shortest_path_length << " pruned, with merged distance: " << (_gg.frontiers.at(mergeable_frontier.frontier).merged_distance + mergeable_frontier.distance_between_frontiers));
						addFrontierToBePruned(remaining_frontier,
								pruned_frontiers, pruned_paths, rrg);
					}
					ROS_INFO_STREAM(
							"Remaining frontier " << remaining_frontier << " replaced with frontier " << mergeable_frontier.frontier << " with length " << mergeable_frontier.path_length_to_local_graph << " with merged distance: " << (_gg.frontiers.at(mergeable_frontier.frontier).merged_distance + mergeable_frontier.distance_between_frontiers));
					remaining_frontier = mergeable_frontier.frontier;
					shortest_path_length =
							mergeable_frontier.path_length_to_local_graph;
					distance_current_frontier_to_remaining_frontier =
							mergeable_frontier.distance_between_frontiers;
				} else {
					ROS_INFO_STREAM(
							"Mergeable frontier " << mergeable_frontier.frontier << " with length " << mergeable_frontier.path_length_to_local_graph << " pruned, with merged distance: " << (_gg.frontiers.at(mergeable_frontier.frontier).merged_distance + mergeable_frontier.distance_between_frontiers));
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
					ROS_INFO_STREAM(
							"Remaining frontier " << remaining_frontier << " pruned because of max merged distance violation, its merged distance is " << max_relevant_merged_distance);
					addFrontierToBePruned(remaining_frontier, pruned_frontiers,
							pruned_paths, rrg);
					frontier.merged_distance =
							merged_distances_to_frontiers.rbegin()->first;
					ROS_INFO_STREAM(
							"Therefore new frontier " << frontier.index << " remains after merge with merged distance " << merged_distances_to_frontiers.rbegin()->first);
				} else { //merge like all other frontiers were merged into current frontier and then into remaining frontier
					_gg.frontiers.at(remaining_frontier).merged_distance =
							std::max(max_relevant_merged_distance,
									_gg.frontiers.at(remaining_frontier).merged_distance);
					ROS_INFO_STREAM(
							"New frontier " << frontier.index << " was merged into existing frontier " << remaining_frontier << " with merged distance " << _gg.frontiers.at(remaining_frontier).merged_distance);
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
				ROS_INFO_STREAM(
						"Retrieved new index for frontier " << frontier.index << " with merged distance " << frontier.merged_distance);
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
	ROS_INFO_STREAM(
			"Check frontier mergeability for frontier " << frontier_one.index << " ("<<frontier_one.merged_distance <<") with path " << path_one.index << " and frontier "<< _gg.paths.at(path_two).frontier << " (" << _gg.frontiers.at(_gg.paths.at(path_two).frontier).merged_distance<<") with path " << path_two << " with combined length " << combined_length << " and distance: " << distance);
	if (_gg.paths.at(path_two).frontier
			!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
			&& combined_length <= 2 * _local_graph_radius
			&& distance <= _local_graph_radius) {
		//to merge frontier 2 into frontier 1: distance + frontier2 merged_distance <= local_graph_radius
		//to merge frontier 1 into frontier 2: distance + frontier1 merged_distance <= local_graph_radius
		//new merged distance is max of

		double frontier_one_new_merged_distance = frontier_one.merged_distance
				+ distance;
		double frontier_two_new_merged_distance = _gg.frontiers.at(
				_gg.paths.at(path_two).frontier).merged_distance + distance;
		if (frontier_one_new_merged_distance <= _local_graph_radius
				|| frontier_two_new_merged_distance <= _local_graph_radius) {
			if (frontier_two_new_merged_distance > _local_graph_radius) { //frontier one won't have the frontiers merged into frontier two in its local graph radius, therefore frontier two is unpruneable
				if (combined_length
						< closest_mergeable_unprunable_frontier.path_length_to_local_graph) {
					ROS_INFO_STREAM(
							"New closest mergeable unprunable frontier " << _gg.paths.at(path_two).frontier << " with length " << _gg.paths.at(path_two).length);
					closest_mergeable_unprunable_frontier =
							MergeableFrontierStruct(
									_gg.paths.at(path_two).frontier,
									_gg.paths.at(path_two).length, distance);
				} else {
					ROS_INFO_STREAM(
							"Frontier " << closest_mergeable_unprunable_frontier.frontier << " is closer, merge not possible");
				}
			} else {
				ROS_INFO_STREAM(
						"Add frontier " << _gg.paths.at(path_two).frontier << " to mergeable frontiers with length " << _gg.paths.at(path_two).length);
				mergeable_frontiers.emplace_back(
						_gg.paths.at(path_two).frontier,
						_gg.paths.at(path_two).length, distance);
			}
		} else {
			ROS_INFO_STREAM(
					"Frontiers cannot be merged due to too large merge distances");
		}
	}
}

void GlobalGraphHandler::connectFrontiers(int frontier_one, int frontier_two,
		int path_one, int path_two, bool connection_at_frontier) {
	ROS_INFO_STREAM(
			"Connect frontiers " << frontier_one << " (" << path_one << ") and " << frontier_two << " (" << path_two << ")");
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
	ROS_INFO_STREAM(
			"Established path " << path_between_frontiers.index << " between frontiers " << path_between_frontiers.frontier << " and " << path_between_frontiers.connection << " with length " << path_between_frontiers.length);
}

int GlobalGraphHandler::availableFrontierIndex() {
	if (!_available_frontiers.empty()) {
		// ROS_INFO_STREAM(
		// 	"Available frontier index " << *_available_frontiers.begin());
		return *_available_frontiers.begin();
	} else {
		// ROS_INFO_STREAM("Frontier index at end " << _gg.frontiers_counter);
		return _gg.frontiers_counter;
	}
}

int GlobalGraphHandler::availablePathIndex() {
	if (!_available_paths.empty()) {
		// ROS_INFO_STREAM("Available path index " << *_available_paths.begin());
		return *_available_paths.begin();
	} else {
		// ROS_INFO_STREAM("Path index at end " << _gg.paths_counter);
		return _gg.paths_counter;
	}
}

void GlobalGraphHandler::addFrontierToBePruned(int frontier_to_prune,
		std::set<int> &pruned_frontiers, std::set<int> &pruned_paths,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("Add frontier to be pruned " << frontier_to_prune);
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
	ROS_INFO_STREAM(
			"Handle pruned frontiers, frontier counter " << _gg.frontiers_counter);
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
					ROS_INFO_STREAM(
							"Remove frontier " << i << " from back of list");
					_gg.frontiers.pop_back();
					_gg.frontiers_counter--;
					removed_frontiers = true;
				} else {
					break;
				}
			}
		} else {
			// ROS_INFO_STREAM("Add available frontier " << *pruned_frontier);
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
		// ROS_WARN_STREAM(
		// 	"Removed " << removals << " available frontiers above equal index " << _gg.frontiers_counter);
	}
}

void GlobalGraphHandler::handlePrunedPaths(const std::set<int> &pruned_paths) {
//	ROS_INFO_STREAM(
//	 	"Handle pruned paths, path counter " << _gg.paths_counter << " ############");
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
					// ROS_INFO_STREAM(
					// 	"Remove path " << i << " from back of list");
					_gg.paths.pop_back();
					_gg.paths_counter--;
					removed_paths = true;
				} else {
					break;
				}
			}
		} else {
			// ROS_INFO_STREAM("Add available path " << *pruned_path);
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
		// ROS_WARN_STREAM(
		// 	"Removed " << removals << " available paths above equal index " << _gg.paths_counter);
	}
}

void GlobalGraphHandler::deactivateFrontier(int pruned_frontier) {
	ROS_INFO_STREAM("Deactivate frontier " << pruned_frontier);
	_gg.frontiers.at(pruned_frontier).inactive = true;
	_gg.frontiers.at(pruned_frontier).merged_distance = 0.0;
	_gg.frontiers.at(pruned_frontier).paths.clear();
	_gg.frontiers.at(pruned_frontier).paths_counter = 0;
	_gg.frontiers.at(pruned_frontier).viewpoint.x = 0;
	_gg.frontiers.at(pruned_frontier).viewpoint.y = 0;
}

void GlobalGraphHandler::deactivatePath(int pruned_path) {
	ROS_INFO_STREAM("Deactivate path " << pruned_path);
	if (_gg.paths.at(pruned_path).frontier < _gg.frontiers_counter
			&& !_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).inactive) { // remove path from list at frontier
		_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.erase(
				std::remove(
						_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.begin(),
						_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.end(),
						pruned_path),
				_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.end());
		// ROS_INFO_STREAM(
		// 	"Removed path from " << _gg.paths.at(pruned_path).frontier << " path list");
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
		// ROS_INFO_STREAM(
		// 	"Removed path from " << _gg.paths.at(pruned_path).connection << " path list");
	}
	_gg.paths.at(pruned_path).inactive = true;
	_gg.paths.at(pruned_path).waypoints.clear();
	_gg.paths.at(pruned_path).length = 0;
}

std::vector<geometry_msgs::Point> GlobalGraphHandler::pruneFrontiersAndPathsAroundNewNode(
		rrg_nbv_exploration_msgs::Graph &rrg, int new_node) {
	ROS_INFO_STREAM(
			"+++++ pruneFrontiersAndPathsAroundNewNode at node " << new_node);
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
	ROS_INFO_STREAM(
			"----- pruneFrontiersAndPathsAroundNewNode at node " << new_node);
	return new_node_positions;
}

std::vector<geometry_msgs::Point> GlobalGraphHandler::pruneFrontiersAroundNewNode(
		int new_node, std::set<int> &pruned_frontiers,
		std::set<int> &pruned_paths, rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("pruneFrontiersAroundNewNode");
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
				ROS_INFO_STREAM(
						"Replace frontier " << frontier_near_new_node.first << " with node " << new_node);
				tryToRewirePathsToLocalGraphOverPrunedFrontier(
						frontier_near_new_node.first, new_node,
						sqrt(frontier_near_new_node.second), rrg);
				addFrontierToBePruned(frontier_near_new_node.first,
						pruned_frontiers, pruned_paths, rrg);
			} else // try to place a node at frontier position to prune it
			{
				ROS_INFO_STREAM(
						"Try to place a node at frontier " << frontier_near_new_node.first << "'s viewpoint to prune it");
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
	ROS_WARN_STREAM(
			"###########tryToRewirePathsToLocalGraphOverPrunedFrontier frontier" << pruned_frontier);
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
				ROS_INFO_STREAM(
						"Replace path "<< other_frontiers_path_to_local_graph << " to RRG with connecting path "<< path << "and new node");
				ROS_INFO_STREAM(
						"Connection to other frontier " << other_frontier << " with path " << other_frontiers_path_to_local_graph << " length: " << _gg.paths.at(other_frontiers_path_to_local_graph).length << " new distance: " << new_distance);
				ROS_INFO_STREAM(
						"Path "<< other_frontiers_path_to_local_graph << " from " << _gg.paths.at(other_frontiers_path_to_local_graph).frontier << " to " << _gg.paths.at(other_frontiers_path_to_local_graph).connection);

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
//					_gg.paths.at(other_frontiers_path_to_local_graph).waypoints =
//							_gg.paths.at(path).waypoints;
				}
				_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.push_back(
						rrg.nodes.at(new_node).position);
				_gg.paths.at(other_frontiers_path_to_local_graph).length =
						new_distance;
				int connecting_node = _gg.paths.at(
						other_frontiers_path_to_local_graph).connecting_node;
				ROS_INFO_STREAM(
						"Set length to " << new_distance << " and replace connecting node " << connecting_node << " with " << new_node);
//				std::string cons = "";
//				for (auto con_path : rrg.nodes.at(connecting_node).connected_to) {
//					cons += std::to_string(con_path) + ",";
//				}
//				ROS_INFO_STREAM(
//						"Connected to node " << connecting_node << " before removal: " << cons << " with size: " << rrg.nodes.at(connecting_node).connected_to.size());
				rrg.nodes.at(connecting_node).connected_to.erase(
						std::remove(
								rrg.nodes.at(connecting_node).connected_to.begin(),
								rrg.nodes.at(connecting_node).connected_to.end(),
								other_frontiers_path_to_local_graph),
						rrg.nodes.at(connecting_node).connected_to.end());
//				auto it = std::find(
//						rrg.nodes.at(connecting_node).connected_to.begin(),
//						rrg.nodes.at(connecting_node).connected_to.end(),
//						other_frontiers_path_to_local_graph);
//				if (it != rrg.nodes.at(connecting_node).connected_to.end()) {
//					rrg.nodes.at(connecting_node).connected_to.erase(it);
//				} else {
//					ROS_WARN_STREAM("Path not found at connecting node!");
//				}
//				ROS_INFO_STREAM(
//						"Connected to at node " << connecting_node << " after removal: " << rrg.nodes.at(connecting_node).connected_to.size());
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
	ROS_INFO_STREAM("prunePathsAroundNewNode " << new_node);
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
			ROS_INFO_STREAM(
					"For path " << path << " to frontier " << frontier.index << ", " << waypoints_near_new_node.size() << " waypoints are nearby");
			int closest_waypoint_to_frontier = getClosestWaypointToFrontier(
					path, new_node, waypoints_near_new_node, rrg);
			if (closest_waypoint_to_frontier
					< (_gg.paths.at(path).waypoints.size() - 1)) {
				// if path can be pruned
				double new_path_length = calculateNewPathLength(path, new_node,
						closest_waypoint_to_frontier, rrg);
//				ROS_INFO_STREAM(
//						"Prev path length: " << _gg.paths.at(path).length << ", new " << new_path_length);
				if (new_path_length < _gg.paths.at(path).length) {
					// only re-route path if length decreases
					ROS_INFO_STREAM(
							"Reduced path " << path << " from frontier " << _gg.paths.at(path).frontier << " to new node " << new_node << " from " << _gg.paths.at(path).length << " to " << new_path_length);
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
	ROS_INFO_STREAM("getClosestWaypointToFrontier");
	int closest_waypoint_to_frontier = _gg.paths.at(path).waypoints.size() - 1; // index of last waypoint in list
// find connectable waypoint closest to the frontier (most path reduction)
	if (_inflation_active) {
		for (auto waypoint_near_new_node : waypoints_near_new_node) {
			//							ROS_INFO_STREAM(
			//									"Waypoint " << waypoint_near_new_node.first << " at distance " << waypoint_near_new_node.second << " (" << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).x << ", " << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).y << ")");
			if (waypoint_near_new_node.first < closest_waypoint_to_frontier) {
				//&& waypoint_near_new_node.first != 0) { // new node must overlap with waypoint at frontier for re-wiring
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
			//							ROS_INFO_STREAM(
			//									"Waypoint " << waypoint_near_new_node.first << " at distance " << waypoint_near_new_node.second << " (" << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).x << ", " << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).y << ")");
			if (//waypoint_near_new_node.first != 0 && // new node must intersect waypoint at frontier for re-wiring or not allow a node to be placed at frontier (min distance)
			_collision_checker->checkConnectionToFrontierPathWaypoint(rrg,
					new_node,
					_gg.paths.at(path).waypoints.at(
							waypoint_near_new_node.first),
					sqrt(waypoint_near_new_node.second))) { // check if a connection can be made
				closest_waypoint_to_frontier = waypoint_near_new_node.first;
				break;
			}
		}
	}
	ROS_INFO_STREAM(
			"Closest waypoint to path " << path << " is " << closest_waypoint_to_frontier);
	return closest_waypoint_to_frontier;
}

double GlobalGraphHandler::calculateNewPathLength(int path, int new_node,
		int closest_waypoint_to_frontier,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM(
			"calculateNewPathLength of path " << path << " to node " << new_node);
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
	ROS_INFO_STREAM(
			"Path " << path << " (" << _gg.paths.at(path).length << ") when connecting to node " << new_node << " from waypoint " << closest_waypoint_to_frontier << " has new length " << new_length << ", distance to new node: " << distance_to_new_node);
	return (new_length + distance_to_new_node);
}

void GlobalGraphHandler::rewirePathToNewNode(int path,
		int closest_waypoint_to_frontier, double new_path_length, int new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM(
			"rewirePathToNewNode: path " << path << " to frontier " << _gg.paths.at(path).frontier << " to new node " << new_node << " with new length " << new_path_length);
	std::string cons = "";
	for (auto con_path : rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to) {
		cons += std::to_string(con_path) + ",";
	}
//	ROS_INFO_STREAM(
//			"Connected to at node " << _gg.paths.at(path).connecting_node << " before removal: " << cons << " with size: " << rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.size());
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
	ROS_INFO_STREAM("tryToImproveConnectionsToOtherFrontiers");
	int current_frontier = _gg.paths.at(path).frontier; // look for new connections with this frontier
	ROS_INFO_STREAM(
			"Try to improve connections from current frontier " << current_frontier << " with path " << path);
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
//		ROS_INFO_STREAM(
//				"Check Path " << frontier_path << " to frontier " << other_frontier);
		if (_gg.paths.at(frontier_path).connection
				!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH
				&& new_frontier_connections.erase(other_frontier) > 0) {
			// remove existing connection from new frontier connections (if present, try to improve path)
			int other_frontier_path_to_local_graph = _gg.frontiers.at(
					other_frontier).paths.front();
			ROS_INFO_STREAM(
					"Existing path "<<frontier_path<< " to " << other_frontier << " with length " << _gg.paths.at(frontier_path).length << " compared to length " << _gg.paths.at(path).length + _gg.paths.at(other_frontier_path_to_local_graph).length);
			if ((_gg.paths.at(path).length
					+ _gg.paths.at(other_frontier_path_to_local_graph).length)
					< _gg.paths.at(frontier_path).length) { // path through new node would be shorter
				ROS_INFO_STREAM(
						"Improve path " << other_frontier_path_to_local_graph << " to existing frontier " << other_frontier << " with length " << _gg.paths.at(frontier_path).length << " compared to new length " << _gg.paths.at(path).length + _gg.paths.at(other_frontier_path_to_local_graph).length);
				improvePathToConnectedFrontier(frontier_path, path,
						other_frontier, other_frontier_path_to_local_graph);

			}
		}
	}
//							ROS_INFO_STREAM(
//									"Add new " << new_frontier_connections.size()<<" connections");
	for (auto new_frontier_connection : new_frontier_connections) {
		connectFrontiers(current_frontier, new_frontier_connection, path,
				_gg.frontiers.at(new_frontier_connection).paths.front(), false); // first path of frontier is always path to local graph
	}
}

void GlobalGraphHandler::improvePathToConnectedFrontier(int frontier_path,
		int path, int other_frontier, int other_path) {
	ROS_INFO_STREAM("improvePathToConnectedFrontier");
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
	ROS_INFO_STREAM("+++++ Calculate Next Frontier Goal");
	std::vector<int> active_frontiers;
	for (auto &frontier : _gg.frontiers) {
		if (!frontier.inactive)
			active_frontiers.push_back(frontier.index);
	}
	if (active_frontiers.size() == 1 && _auto_homing) { // only origin remains
		ROS_INFO_STREAM("only one frontier remaining->origin");
		_global_route.push_back(std::make_pair(0, 0));
		_next_global_goal = 0;
	} else if (active_frontiers.size() <= 1) {
		ROS_INFO_STREAM("----- Calculate Next Frontier Goal, no frontier");
		return false;
	} else {
		establishMissingFrontierToFrontierConnections(active_frontiers, rrg);
		connectPathsToLocalGraphToNearestNode(rrg);
		std::pair<int, int> next_frontier_with_path =
				findBestFrontierWithTspTwoOpt(active_frontiers);
		if (next_frontier_with_path.first == -1
				|| next_frontier_with_path.second == -1) {
			ROS_INFO_STREAM(
					"----- Calculate Next Frontier Goal, no frontier found with 2-Opt");
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
	ROS_INFO_STREAM("----- Calculate Next Frontier Goal");
	return true;
}

std::pair<int, int> GlobalGraphHandler::findBestFrontierWithTspTwoOpt(
		std::vector<int> &active_frontiers) {
	ROS_INFO_STREAM("+++++ Start TSP 2-opt");
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
			ROS_WARN_STREAM(
					"Found no path from active frontier " << active_frontiers.at(i - 1) << " to " << active_frontiers.at(i));
			return std::make_pair(-1, -1);
		}
		route.push_back(std::make_pair(active_frontiers.at(i), path_index));
	}
	std::string route_info = "Route: ";
	for (auto n : route) {
		route_info += std::to_string(n.first) + " (" + std::to_string(n.second)
				+ ")->";
	}
	route_info += " with distance="
			+ std::to_string(calculateRouteLength(route));
	ROS_INFO_STREAM(route_info);
	ROS_INFO_STREAM("Calculated edges");
	bool improved = true;
	while (improved) {
		improved = iterateOverTwoOptSwaps(route);
	}
	route_info = "No improvement found, route: ";
	for (auto n : route) {
		route_info += std::to_string(n.first) + " (" + std::to_string(n.second)
				+ ")->";
	}
	route_info += " with distance="
			+ std::to_string(calculateRouteLength(route));
	ROS_INFO_STREAM(route_info);
	ROS_INFO_STREAM("----- End TSP 2-opt");
	_global_route = route;
	return std::make_pair(route.at(1).first, route.at(1).second);
}

bool GlobalGraphHandler::iterateOverTwoOptSwaps(
		std::vector<std::pair<int, int>> &route) {
	double best_distance = calculateRouteLength(route);
//	ROS_INFO_STREAM("Start distance: " << best_distance);
	for (int i = 1; i < route.size() - 1; i++) {
// omit first frontier (robot position/local graph)
		for (int k = i + 1; k < route.size() - (_auto_homing ? 1 : 0); k++) {
			// omit last frontier (origin) when auto homing
			std::vector<std::pair<int, int>> new_route;
			if (twoOptSwap(route, new_route, i, k)) {
				// if swap resulted in traversable route
				double new_distance = calculateRouteLength(new_route);
//				ROS_INFO_STREAM(
//						"Swapped " << i << " to " << k << " new distance: " << new_distance);
//				std::string route_info = "Swapped route: ";
//				for (auto n : new_route) {
//					route_info += std::to_string(n.first) + " ("
//							+ std::to_string(n.second) + ")->";
//				}
//				ROS_INFO_STREAM(route_info);
				if (new_distance < best_distance) {
					route = new_route;
					best_distance = new_distance;
//					route_info = "Swap improved route: ";
//					for (auto n : route) {
//						route_info += std::to_string(n.first) + " ("
//								+ std::to_string(n.second) + ")->";
//					}
//					route_info += " with distance="
//							+ std::to_string(best_distance);
//					ROS_INFO_STREAM(route_info);
					return true;
				}
			}
		}
	}
	return false;
}

int GlobalGraphHandler::findPathToNextFrontier(int current_frontier,
		int next_frontier) {
//	ROS_INFO_STREAM(
//			"Find path to next frontier from " << current_frontier << " to " << next_frontier);
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
//	ROS_INFO_STREAM("Found path " << path_index);
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
//	ROS_INFO_STREAM("Two opt swap of " << i << " and " << k);
	for (int j = 0; j < i; j++) {
		new_route.push_back(route.at(j));
	}
//	ROS_INFO_STREAM("Start reversing");
	for (int l = k; l >= i; l--) {
		int path_index = findPathToNextFrontier(new_route.back().first,
				route.at(l).first);
		new_route.push_back(route.at(l));
		new_route.back().second = path_index;
	}
//	ROS_INFO_STREAM("Finished reversing");
	for (int m = k + 1; m < route.size(); m++) {
		new_route.push_back(route.at(m));
	}
//	ROS_INFO_STREAM("Two opt swap of " << i << " and " << k << " finished");
	return true;
}

void GlobalGraphHandler::establishMissingFrontierToFrontierConnections(
		std::vector<int> active_frontiers,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM(
			"Connect all frontiers with nearest node " << rrg.nearest_node << " and each other");
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
			for (auto missing_frontier : missing_frontiers) {
				// build the missing paths
				if (frontier.index > missing_frontier) { // only build paths where this frontier will be frontier in path
					ROS_INFO_STREAM(
							"Establish missing connection between frontiers " << frontier.index << " and " << missing_frontier);
					int missing_frontier_connecting_node =
							_gg.paths.at(
									_gg.frontiers.at(missing_frontier).paths.front()).connecting_node;
					ROS_INFO_STREAM(
							"Frontier connecting node " << frontier_connecting_node << ", missing frontier connecting node " << missing_frontier_connecting_node);
					bool found_existing_connection = false;
					for (auto frontier_connection : frontier_connections) {
						if (frontier_connection.connecting_node_one
								== std::max(frontier_connecting_node,
										missing_frontier_connecting_node)
								&& frontier_connection.connecting_node_two
										== std::min(frontier_connecting_node,
												missing_frontier_connecting_node)) {
							ROS_INFO_STREAM(
									"Reuse existing connection between " << frontier_connection.connecting_node_one << " and " << frontier_connection.connecting_node_two);
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
	ROS_INFO_STREAM("established MissingFrontierToFrontierConnections");
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
		ROS_INFO_STREAM(
				"Check if node " << *rit << " from frontier's path to robot is also present in missing frontier's path to robot");
		auto it =
				std::find(
						std::next(
								rrg.nodes.at(missing_frontier_connecting_node).path_to_robot.begin()), // first node is connecting node
						rrg.nodes.at(missing_frontier_connecting_node).path_to_robot.end(),
						*rit); // find occurence of node in both paths
		if (it
				!= rrg.nodes.at(missing_frontier_connecting_node).path_to_robot.end()) { // found mutual node in paths
			ROS_INFO_STREAM("First mutual node in local RRG is " << *it);
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
			ROS_INFO_STREAM("Remaining path in local RRG is: " << rempa);
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
				ROS_INFO_STREAM(
						"Minimum local path length at mutual node " << *it << " is " << minimum_local_path_length);
				break;
			}
			ROS_INFO_STREAM("Continue looking for mutual node");
		}
	}
	if (mutual_node_local_path.empty()) {
		ROS_INFO_STREAM("No mutual node, use full paths to robot");
		max_distance_threshold = std::numeric_limits<double>::infinity(); // cannot use distance threshold
	}
	max_distance_threshold = std::max(max_distance_threshold,
			minimum_local_path_length);
}

void GlobalGraphHandler::buildMissingPathBetweenFrontiers(
		rrg_nbv_exploration_msgs::GlobalFrontier_<std::allocator<void>> &frontier,
		int missing_frontier, double path_length, std::vector<int> &local_path,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM(
			"Build path between " << frontier.index << " and " << missing_frontier);
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
	ROS_INFO_STREAM(
			"Established path " << path_between_frontiers.index << " between frontiers " << path_between_frontiers.frontier << " and " << path_between_frontiers.connection << " with length " << path_between_frontiers.length);
}

void GlobalGraphHandler::connectPathsToLocalGraphToNearestNode(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("connectPathsToLocalGraphToNearestNode");
	for (auto &path : _gg.paths) {
// connect all frontiers directly to the nearest node to the robot
		if (!path.inactive
				&& path.connection
						== rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) {
			// add path to robot from connecting node
			for (int i = rrg.nodes.at(path.connecting_node).path_to_robot.size()
					- 2; i >= 0; i--) {
				// skip node itself, as it already is in waypoints
				path.waypoints.push_back(
						rrg.nodes.at(
								rrg.nodes.at(path.connecting_node).path_to_robot.at(
										i)).position);
			}
			path.length += rrg.nodes.at(path.connecting_node).distance_to_robot;
			ROS_INFO_STREAM(
					"Path " << path.index << " to frontier " << path.frontier << " connected to node " << rrg.nearest_node << " with length " << path.length);
			path.connecting_node = rrg.nearest_node;
		}
	}
	ROS_INFO_STREAM(
			"Connected all frontiers directly to nearest node " << rrg.nearest_node);
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
			//			int waypoint_index =
			//					_gg.paths.at(_global_route.at(_next_global_goal).second).frontier
			//							== _global_route.at(_next_global_goal).first ?
			//							1 :
			//							*std::prev(
			//									_gg.paths.at(
			//											_global_route.at(_next_global_goal).second).waypoints.end(),
			//									2); //if frontier goal is frontier in path, second waypoint index is relevant for yaw, otherwise second to last (frontier is always first waypoint, connection last)
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
		ROS_INFO_STREAM(
				"Unable to retrieve frontier goal because it is not in the list");
		return false;
	}
}

bool GlobalGraphHandler::getFrontierPath(
		std::vector<geometry_msgs::PoseStamped> &path,
		geometry_msgs::Point &robot_pos) {
	if (checkIfNextFrontierWithPathIsValid()) {
// ROS_INFO_STREAM("Get frontier path");
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
		ROS_INFO_STREAM(
				"Unable to retrieve frontier path because it is not in the list");
		return false;
	}
}

std::vector<int> GlobalGraphHandler::frontierReached(
		geometry_msgs::Point &position) {
	ROS_INFO_STREAM("+++++ frontierReached");
	std::vector<int> connected_paths;
	if (checkIfNextFrontierWithPathIsValid()) {
		int frontier = _global_route.at(_next_global_goal).first;
		position = _gg.frontiers.at(frontier).viewpoint;
		ROS_INFO_STREAM("Frontier " << frontier);
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
		ROS_INFO_STREAM("Prune frontier and paths");
		if (pruned_frontiers.size() > 0) {
			handlePrunedFrontiers(pruned_frontiers);
			handlePrunedPaths(pruned_paths);
		}
	} else {
		ROS_INFO_STREAM("Unable to retrieve frontier and path");
	}
	return connected_paths;
	ROS_INFO_STREAM("----- frontierReached");
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
		ROS_INFO_STREAM(
				"Overwrite path to local graph " << other_frontiers_path_to_local_graph << "of frontier " << other_frontier << " with path " << path << " and length " << _gg.paths.at( path).length);
// overwrite path between other frontier and local graph with details from the path between those two
		_gg.paths.at(other_frontiers_path_to_local_graph).connecting_node = 0; // root at new RRG
		_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.clear();
		if (other_frontier > frontier) {
			_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.insert(
					_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.end(),
					_gg.paths.at(path).waypoints.begin(),
					_gg.paths.at(path).waypoints.end());
		} else {
			// path was from frontier to other frontier, must be reversed
			_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.insert(
					_gg.paths.at(other_frontiers_path_to_local_graph).waypoints.end(),
					_gg.paths.at(path).waypoints.rbegin(),
					_gg.paths.at(path).waypoints.rend());
		}
		_gg.paths.at(other_frontiers_path_to_local_graph).length = _gg.paths.at(
				path).length;
//		_gg.paths.at(path).connecting_node = -1; // prevents erase at connecting node when pruning frontier
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
		ROS_INFO_STREAM("New closest waypoint is " << nearest_node);
		if (nearest_node == 0) { // frontier reached
			_active_paths_closest_waypoint = std::make_pair(-1, -1);
			return true;
		} else if (_previous_global_goal_failed
				&& nearest_node
						== _gg.paths.at(_active_paths_closest_waypoint.first).waypoints.size()
								- 1) { // reached start of path, transfer to path to next frontier
			ROS_INFO_STREAM(
					"Reached start of path to failed frontier, follow path to next frontier");
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
	_local_graph_radius_squared = pow(_local_graph_radius, 2);
}

} /* namespace rrg_nbv_exploration */
