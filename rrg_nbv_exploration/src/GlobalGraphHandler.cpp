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
	private_nh.param("sensor_max_range", _sensor_range, 5.0);
	_sensor_range_squared = pow(_sensor_range, 2);
	private_nh.param("inflation_active", _inflation_active, true);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("robot_width", _robot_width, 1.0);
	_path_box_distance_thres = sqrt(
			pow(_robot_radius, 2) - pow(_robot_width / 2, 2));
	double max_edge_distance;
	private_nh.param("max_edge_distance", max_edge_distance, 1.0);
	_max_edge_distance_squared = pow(max_edge_distance, 2);
	_robot_radius_squared = pow(_robot_radius, 2);
	ros::NodeHandle nh("rne");
	_global_graph_publisher =
			nh.advertise<rrg_nbv_exploration_msgs::GlobalGraph>("globalgraph",
					1);

	_global_graph_searcher.reset(new GlobalGraphSearcher());
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
	rrg_nbv_exploration_msgs::GlobalGraph test2 = _gg;
	_global_graph_searcher->initialize(_gg);
}

void GlobalGraphHandler::publishGlobalGraph() {
	_global_graph_publisher.publish(_gg);
}

bool GlobalGraphHandler::getConnectingNode(int node,
		rrg_nbv_exploration_msgs::Graph &rrg,
		std::vector<geometry_msgs::Point> &waypoints, int &connecting_node,
		double &length) {
	if (rrg.nodes[node].path_to_robot.size() > 1) {
		int next_node = rrg.nodes[node].path_to_robot.size() - 2;
//		ROS_INFO_STREAM(
//				"Path to robot at index " << next_node << " is " << rrg.nodes[node].path_to_robot.at(next_node));
		int edge = _graph_path_calculator->findExistingEdge(rrg,
				rrg.nodes[node].path_to_robot.at(next_node + 1),
				rrg.nodes[node].path_to_robot.at(next_node));
//		ROS_INFO_STREAM(
//				"Edge found from node " << rrg.nodes[node].path_to_robot.at(next_node + 1) << " to " << rrg.nodes[node].path_to_robot.at(next_node) << ": " << edge);
		if (edge != -1) {
			length += rrg.edges[edge].length;
//			ROS_INFO_STREAM(
//					"Add edge " << edge << " to path with length " << rrg.edges[edge].length << " total: " << length);
		}
		waypoints.push_back(
				rrg.nodes[rrg.nodes[node].path_to_robot.at(next_node)].position);
		connecting_node = rrg.nodes[node].path_to_robot.at(next_node);
		return true;
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
	frontier.viewpoint = rrg.nodes[node].position;
	path.connection = rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH;
	path.waypoints.push_back(rrg.nodes[node].position);
	ROS_INFO_STREAM(
			"##### Add frontier " << frontier.index<<" at node " << node);
	if (!getConnectingNode(node, rrg, path.waypoints, path.connecting_node,
			path.length)) {
		return;
	}
	if (!mergeNeighborFrontiers(node, path, rrg, frontier)) { //if new frontier is merged into existing
		return;
	}
	path.index = availablePathIndex();
	path.frontier = frontier.index;
	frontier.paths.push_back(path.index);
	frontier.paths_counter++;
	insertFrontierInGg(frontier);
	insertPathInGg(path);
	if (!rrg.nodes[node].connected_to.empty()) { //new frontier is in path of existing frontiers
		for (auto connected_path : rrg.nodes[node].connected_to) {
			connectFrontiers(frontier.index,
					_gg.paths.at(connected_path).frontier, path, connected_path,
					false);
		}
	}
	if (!rrg.nodes[path.connecting_node].connected_to.empty()) { //connecting node is in path of existing frontiers, connect frontiers
		for (auto connected_path : rrg.nodes[path.connecting_node].connected_to) {
			rrg_nbv_exploration_msgs::GlobalPath path_between_frontiers;
			connectFrontiers(frontier.index,
					_gg.paths.at(connected_path).frontier, path, connected_path,
					true);
		}
	}
	rrg.nodes[path.connecting_node].connected_to.push_back(path.index);
	_global_graph_searcher->rebuildIndex(_gg);
	//TODO: check if path ends are back in local graph and remove them/connect them to new nearest node
}

void GlobalGraphHandler::continuePath(int node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<geometry_msgs::Point> additional_waypoints;
	int connecting_node;
	ROS_INFO_STREAM("##### Continue path at node " << node);
	double length = 0;
	if (!getConnectingNode(node, rrg, additional_waypoints, connecting_node,
			length)) {
		return;
	}
	std::map<int, int> connecting_node_frontiers; //frontier index (first), path to connected node index (second)
	for (auto connected_path : rrg.nodes[connecting_node].connected_to) { //get list of already present path connections at new connected node in local graph
		connecting_node_frontiers.insert( { _gg.paths[connected_path].frontier,
				connected_path });
	}
	for (int path : rrg.nodes[node].connected_to) {
		_gg.paths[path].waypoints.insert(_gg.paths[path].waypoints.end(),
				additional_waypoints.begin(), additional_waypoints.end()); //add additional waypoints to path
		_gg.paths[path].connecting_node = connecting_node;
		_gg.paths[path].length += length;
		if (std::find(
				rrg.nodes[_gg.paths[path].connecting_node].connected_to.begin(),
				rrg.nodes[_gg.paths[path].connecting_node].connected_to.end(),
				path) //add connected path to node if not already present
		== rrg.nodes[_gg.paths[path].connecting_node].connected_to.end()) {
			rrg.nodes[_gg.paths[path].connecting_node].connected_to.push_back(
					path);
		}
		if (connecting_node_frontiers.size() > 0) {
			int current_frontier = _gg.paths[path].frontier; //look for new connections with this frontier
			std::set<int> new_frontier_connections; //set of existing connected frontiers to the connecting node which will be reduced by the frontiers already connected to the current frontier
			for (auto elem : connecting_node_frontiers) {
				new_frontier_connections.insert(elem.first);
			}
			for (auto frontier_path : _gg.frontiers[current_frontier].paths) {
				if (_gg.paths[frontier_path].connection
						!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) { //connected to another frontier
					new_frontier_connections.erase(
							_gg.paths[frontier_path].connection); //remove from new frontiers (if present)
				}
			}
			for (auto new_frontier_connection : new_frontier_connections) {
				connectFrontiers(current_frontier, new_frontier_connection,
						_gg.paths.at(path),
						_gg.paths.at(
								connecting_node_frontiers.at(
										new_frontier_connection)).index, true);
			}
		}
	}
}

bool GlobalGraphHandler::mergeNeighborFrontiers(int node,
		const rrg_nbv_exploration_msgs::GlobalPath &path,
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::GlobalFrontier &frontier) {
	if (!rrg.nodes[node].connected_to.empty()
			|| !rrg.nodes[path.connecting_node].connected_to.empty()) {
		std::vector<std::pair<int, double> > mergeable_frontiers; //frontier index first and distance to local graph second
		for (auto connected_path : rrg.nodes[node].connected_to) {
			if (_gg.paths.at(connected_path).frontier
					!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
					&& _gg.paths.at(connected_path).waypoints.size() <= 2) { //frontier directly next to node
				mergeable_frontiers.push_back(
						std::make_pair(_gg.paths.at(connected_path).frontier,
								_gg.paths.at(connected_path).length
										+ path.length)); //add this path's length to the connecting node because it is missing for the existing frontier
			}
		}
		for (auto connected_path : rrg.nodes[path.connecting_node].connected_to) {
			if (_gg.paths.at(connected_path).frontier
					!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
					&& _gg.paths.at(connected_path).waypoints.size() <= 2) { //frontier next to connected node
				mergeable_frontiers.push_back(
						std::make_pair(_gg.paths.at(connected_path).frontier,
								_gg.paths.at(connected_path).length));
			}
		}
		if (mergeable_frontiers.size()) {
			std::set<int> pruned_frontiers;
			std::set<int> pruned_paths;
			int remaining_frontier = frontier.index;
			double shortest_path_length = path.length;
			for (auto mergeable_frontier : mergeable_frontiers) { //find remaining frontier
				if (mergeable_frontier.second <= shortest_path_length) {
					if (remaining_frontier != frontier.index) { //not in graph, cannot be pruned
						addFrontierToBePruned(remaining_frontier,
								pruned_frontiers, pruned_paths, rrg);
					}
					remaining_frontier = mergeable_frontier.first;
					shortest_path_length = mergeable_frontier.second;
				} else {
					addFrontierToBePruned(mergeable_frontier.first,
							pruned_frontiers, pruned_paths, rrg);
				}
			}
			handlePrunedFrontiers(pruned_frontiers);
			handlePrunedPaths(pruned_paths);
			if (remaining_frontier != frontier.index) { //don't add frontier, was merged into existing
				ROS_INFO_STREAM(
						"New frontier " << frontier.index << " was merged into existing");
				return false;
			} else { //check if new index became available
				frontier.index = availableFrontierIndex();
				ROS_INFO_STREAM(
						"Retrieved new frontier index "<<frontier.index);
			}
		}
	}
	return true;
}

void GlobalGraphHandler::connectFrontiers(int frontier_one, int frontier_two,
		rrg_nbv_exploration_msgs::GlobalPath &path, int path_at_node,
		bool connecting_node) {
	rrg_nbv_exploration_msgs::GlobalPath path_between_frontiers;
	path_between_frontiers.index = availablePathIndex();
	path_between_frontiers.frontier = std::max(frontier_one, frontier_two);
	path_between_frontiers.connection = std::min(frontier_one, frontier_two);
	if (connecting_node)
		path_between_frontiers.waypoints.insert(
				path_between_frontiers.waypoints.begin(),
				path.waypoints.begin(), path.waypoints.end() - 1); //omit last element to avoid duplicate waypoint
	path_between_frontiers.waypoints.insert(
			path_between_frontiers.waypoints.end(),
			_gg.paths.at(path_at_node).waypoints.rbegin(),
			_gg.paths.at(path_at_node).waypoints.rend());
	path_between_frontiers.length =
			_gg.paths.at(path_at_node).length + connecting_node ?
					+path.length : 0;
	path_between_frontiers.connecting_node = -1;
	_gg.frontiers.at(frontier_one).paths.push_back(
			path_between_frontiers.index);
	_gg.frontiers.at(frontier_one).paths_counter++;
	_gg.frontiers.at(frontier_two).paths.push_back(
			path_between_frontiers.index);
	_gg.frontiers.at(frontier_two).paths_counter++;
	insertPathInGg(path_between_frontiers);
	ROS_INFO_STREAM(
			"established path " <<path_between_frontiers.index << " between frontiers " << path_between_frontiers.frontier << " and " << path_between_frontiers.connection << " with length " << path_between_frontiers.length);
}

int GlobalGraphHandler::availableFrontierIndex() {
	if (_available_frontiers.empty()) {
		return _gg.frontiers_counter;
	} else {
		return *_available_frontiers.begin();
	}
}

int GlobalGraphHandler::availablePathIndex() {
	if (_available_paths.empty()) {
		return _gg.paths_counter;
	} else {
		return *_available_paths.begin();
	}
}

void GlobalGraphHandler::deactivateFrontiersInLocalGraph(
		rrg_nbv_exploration_msgs::Graph &rrg,
		geometry_msgs::Point robot_position) {
	std::set<int> pruned_frontiers;
	std::set<int> pruned_paths;
	std::vector<std::pair<int, double>> frontiers_in_sensor_range =
			_global_graph_searcher->searchInRadius(robot_position,
					_sensor_range_squared); //find all frontiers in sensor range around the robot
	for (auto frontier_in_sensor_range : frontiers_in_sensor_range) {
		if (frontier_in_sensor_range.first
				!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
				&& !_gg.frontiers[frontier_in_sensor_range.first].inactive) { //never prune origin and ignore inactive frontiers
			std::vector<std::pair<int, double>> nodes_in_range =
					_graph_searcher->searchInRadius(
							_gg.frontiers[frontier_in_sensor_range.first].viewpoint,
							_inflation_active ?
									pow(_robot_radius + rrg.largest_node_radius,
											2) :
									_max_edge_distance_squared);
			for (auto node_in_range : nodes_in_range) {
				double compare_distance =
						_inflation_active ?
								pow(
										sqrt(
												rrg.nodes[node_in_range.first].squared_radius
														- pow(_robot_width / 2,
																2))
												+ _path_box_distance_thres, 2) :
								_robot_radius_squared;
				if (node_in_range.second <= compare_distance
						|| (!_inflation_active
								&& _collision_checker->checkConnectionToFrontier(
										rrg, node_in_range.first,
										_gg.frontiers[frontier_in_sensor_range.first].viewpoint,
										sqrt(node_in_range.second)))) { //node overlaps enough with frontier with robot radius
					addFrontierToBePruned(frontier_in_sensor_range.first,
							pruned_frontiers, pruned_paths, rrg);
					break;
				}
			}
		}
	}
	handlePrunedFrontiers(pruned_frontiers);
	handlePrunedPaths(pruned_paths);
}

void GlobalGraphHandler::addFrontierToBePruned(int frontier_to_prune,
		std::set<int> &pruned_frontiers, std::set<int> &pruned_paths,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	pruned_frontiers.insert(frontier_to_prune);
	for (auto path : _gg.frontiers[frontier_to_prune].paths) {
		pruned_paths.insert(path);
		if (_gg.paths[path].connecting_node >= 0) //remove possible connection from node in RRG
			rrg.nodes[_gg.paths[path].connecting_node].connected_to.erase(
					std::remove(
							rrg.nodes[_gg.paths[path].connecting_node].connected_to.begin(),
							rrg.nodes[_gg.paths[path].connecting_node].connected_to.end(),
							path),
					rrg.nodes[_gg.paths[path].connecting_node].connected_to.end());
	}
}

void GlobalGraphHandler::handlePrunedFrontiers(
		const std::set<int> &pruned_frontiers) {
	auto pruned_frontier = pruned_frontiers.rbegin();
	while (pruned_frontier != pruned_frontiers.rend()) { //remove or add inactive frontiers
		if (*pruned_frontier == _gg.frontiers_counter - 1) {
			_gg.frontiers.pop_back();
			_gg.frontiers_counter--;
		} else {
			_available_frontiers.insert(*pruned_frontier);
		}
		deactivateFrontier(*pruned_frontier);
		pruned_frontier++;
	}
}

void GlobalGraphHandler::handlePrunedPaths(const std::set<int> &pruned_paths) {
	auto pruned_path = pruned_paths.rbegin();
	while (pruned_path != pruned_paths.rend()) { //remove or add inactive paths
		if (*pruned_path == _gg.paths_counter - 1) {
			_gg.paths.pop_back();
			_gg.paths_counter--;
		} else {
			_available_paths.insert(*pruned_path);
		}
		deactivatePath(*pruned_path);
		pruned_path++;
	}
}

void GlobalGraphHandler::deactivateFrontier(int pruned_frontier) {
	_gg.frontiers[pruned_frontier].inactive = true;
	_gg.frontiers[pruned_frontier].paths.clear();
	_gg.frontiers[pruned_frontier].paths_counter = 0;
	_gg.frontiers[pruned_frontier].viewpoint.x = 0;
	_gg.frontiers[pruned_frontier].viewpoint.y = 0;
}

void GlobalGraphHandler::deactivatePath(int pruned_path) {
	_gg.paths[pruned_path].inactive = true;
	_gg.paths[pruned_path].waypoints.clear();
	_gg.paths[pruned_path].length = 0;
}

} /* namespace rrg_nbv_exploration */
