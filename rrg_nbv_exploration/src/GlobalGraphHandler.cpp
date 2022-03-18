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
	_local_graph_radius_squared = pow(_local_graph_radius, 2);
	private_nh.param("inflation_active", _inflation_active, true);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("robot_width", _robot_width, 1.0);
	_path_box_distance_thres = sqrt(
			pow(_robot_radius, 2) - pow(_robot_width / 2, 2));
	double max_edge_distance;
	private_nh.param("max_edge_distance", max_edge_distance, 1.0);
	_max_edge_distance_squared = pow(max_edge_distance, 2);
	_robot_radius_squared = pow(_robot_radius, 2);
	private_nh.param("auto_homing", _auto_homing, false);
	ros::NodeHandle nh("rne");
	_global_graph_publisher =
			nh.advertise<rrg_nbv_exploration_msgs::GlobalGraph>("globalgraph",
					1);

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
	_next_frontier_with_path = std::make_pair(-1, -1);
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
	ROS_INFO_STREAM("Get connecting node for node " << node);
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
			return true;
		}
	}
	return false;
}

void GlobalGraphHandler::insertFrontierInGg(
		const rrg_nbv_exploration_msgs::GlobalFrontier &frontier) {
	if (!_available_frontiers.empty()) {
		ROS_INFO_STREAM("Insert frontier at " << *_available_paths.begin());
		_gg.frontiers.at(*_available_frontiers.begin()) = frontier;
		_available_frontiers.erase(_available_frontiers.begin());
	} else {
		ROS_INFO_STREAM("Push frontier at " << _gg.paths_counter);
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
	path.connection = rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH;
	path.waypoints.push_back(rrg.nodes.at(node).position);
	ROS_INFO_STREAM(
			"+++++ Add frontier " << frontier.index<<" at node " << node);
	if (!getConnectingNode(node, rrg, path.waypoints, path.connecting_node,
			path.length)) {
		ROS_WARN_STREAM(
				"Unable to add frontier for node " << node << ", found no connection to the RRG");
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
	if (!rrg.nodes.at(node).connected_to.empty()) { //new frontier is in path of existing frontiers
		for (auto connected_path : rrg.nodes.at(node).connected_to) {
			connectFrontiers(frontier.index,
					_gg.paths.at(connected_path).frontier, path.index,
					connected_path, false);
		}
	}
	if (!rrg.nodes.at(path.connecting_node).connected_to.empty()) { //connecting node is in path of existing frontiers, connect frontiers
		for (auto connected_path : rrg.nodes.at(path.connecting_node).connected_to) {
			connectFrontiers(frontier.index,
					_gg.paths.at(connected_path).frontier, path.index,
					connected_path, true);
		}
	}
	rrg.nodes.at(path.connecting_node).connected_to.push_back(path.index);
	_global_graph_searcher->rebuildIndex(_gg);
	ROS_INFO_STREAM(
			"----- Added frontier " << frontier.index<<" at node " << node);
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
	std::map<int, int> connecting_node_frontiers; //frontier index (first), path to connected node index (second)
	for (auto connected_path : rrg.nodes.at(connecting_node).connected_to) { //get list of already present path connections at new connected node in local graph
		connecting_node_frontiers.insert(
				{ _gg.paths.at(connected_path).frontier, connected_path });
	}
	for (int path : rrg.nodes.at(node).connected_to) {
		ROS_INFO_STREAM(
				"Check path " << path << " that is connected to " << node);
		_gg.paths.at(path).waypoints.insert(_gg.paths.at(path).waypoints.end(),
				additional_waypoints.begin(), additional_waypoints.end()); //add additional waypoints to path
		_gg.paths.at(path).connecting_node = connecting_node;
		_gg.paths.at(path).length += length;
		if (std::find(
				rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.begin(),
				rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.end(),
				path) //add connected path to node if not already present
				== rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.end()) {
			rrg.nodes.at(_gg.paths.at(path).connecting_node).connected_to.push_back(
					path);
		}
		if (connecting_node_frontiers.size() > 0) {
			int current_frontier = _gg.paths.at(path).frontier; //look for new connections with this frontier
			std::set<int> new_frontier_connections; //set of existing connected frontiers to the connecting node which will be reduced by the frontiers already connected to the current frontier
			for (auto elem : connecting_node_frontiers) {
				new_frontier_connections.insert(elem.first);
			}
			for (auto frontier_path : _gg.frontiers.at(current_frontier).paths) {
				ROS_INFO_STREAM(
						"Look at path "<<frontier_path<< " at current frontier " << current_frontier);
				if (_gg.paths.at(frontier_path).connection
						!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) { //connected to another frontier
					new_frontier_connections.erase(
							current_frontier //current frontier can be frontier or connection at connecting path
							== _gg.paths.at(frontier_path).frontier ?
									_gg.paths.at(frontier_path).connection :
									_gg.paths.at(frontier_path).frontier); //remove from new frontiers (if present)
				}
			}
			for (auto new_frontier_connection : new_frontier_connections) {
				connectFrontiers(current_frontier, new_frontier_connection,
						path,
						connecting_node_frontiers.at(new_frontier_connection),
						true);
			}
		}
	}
	ROS_INFO_STREAM("----- Continued path at node " << node);
}

bool GlobalGraphHandler::mergeNeighborFrontiers(int node,
		const rrg_nbv_exploration_msgs::GlobalPath &path,
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::GlobalFrontier &frontier) {
	if (!rrg.nodes.at(node).connected_to.empty()
			|| !rrg.nodes.at(path.connecting_node).connected_to.empty()) {
		ROS_INFO_STREAM(
				"mergeNeighborFrontiers for frontier " << frontier.index);
		std::vector<std::pair<int, double> > mergeable_frontiers; //frontier index first and distance to local graph second
		for (auto connected_path : rrg.nodes.at(node).connected_to) {
			if (_gg.paths.at(connected_path).frontier
					!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
					&& _gg.paths.at(connected_path).waypoints.size() <= 2) { //frontier directly next to node
				mergeable_frontiers.push_back(
						std::make_pair(_gg.paths.at(connected_path).frontier,
								_gg.paths.at(connected_path).length
										+ path.length)); //add this path's length to the connecting node because it is missing for the existing frontier
			}
		}
		for (auto connected_path : rrg.nodes.at(path.connecting_node).connected_to) {
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
			if (pruned_frontiers.size() > 0) {
				handlePrunedFrontiers(pruned_frontiers);
				handlePrunedPaths(pruned_paths);
			}
			if (remaining_frontier != frontier.index) { //don't add frontier, it was merged into existing
				ROS_INFO_STREAM(
						"New frontier " << frontier.index << " was merged into existing frontier " << remaining_frontier);
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
		int path_one, int path_two, bool connecting_node) {
	ROS_INFO_STREAM(
			"Connect frontiers " << frontier_one << " (" <<path_one << ") and " << frontier_two << " (" << path_two << ")");
	int frontier;
	int connection;
	int frontier_path;
	int connection_path;
	if (frontier_one > frontier_two) { //higher index frontier is always frontier, the other one connection, path always starts at frontier
		frontier = frontier_one;
		connection = frontier_two;
		frontier_path = path_one;
		connection_path = path_two;
	} else {
		frontier = frontier_two;
		connection = frontier_one;
		frontier_path = path_two;
		connection_path = path_one;
	}
	rrg_nbv_exploration_msgs::GlobalPath path_between_frontiers;
	path_between_frontiers.index = availablePathIndex();
	path_between_frontiers.frontier = frontier;
	path_between_frontiers.connection = connection;
	if (connecting_node)
		path_between_frontiers.waypoints.insert(
				path_between_frontiers.waypoints.begin(),
				_gg.paths.at(frontier_path).waypoints.begin(),
				_gg.paths.at(frontier_path).waypoints.end() - 1); //omit last element to avoid duplicate waypoint
	path_between_frontiers.waypoints.insert(
			path_between_frontiers.waypoints.end(),
			_gg.paths.at(connection_path).waypoints.rbegin(),
			_gg.paths.at(connection_path).waypoints.rend());
	path_between_frontiers.length = _gg.paths.at(frontier_path).length
			+ (connecting_node ? _gg.paths.at(connection_path).length : 0);
	path_between_frontiers.connecting_node = -1;
	_gg.frontiers.at(frontier).paths.push_back(path_between_frontiers.index);
	_gg.frontiers.at(frontier).paths_counter++;
	_gg.frontiers.at(connection).paths.push_back(path_between_frontiers.index);
	_gg.frontiers.at(connection).paths_counter++;
	insertPathInGg(path_between_frontiers);
	ROS_INFO_STREAM(
			"Established path " <<path_between_frontiers.index << " between frontiers " << path_between_frontiers.frontier << " and " << path_between_frontiers.connection << " with length " << path_between_frontiers.length);
}

int GlobalGraphHandler::availableFrontierIndex() {
	if (!_available_frontiers.empty()) {
		ROS_INFO_STREAM(
				"Available frontier index " << *_available_frontiers.begin());
		return *_available_frontiers.begin();
	} else {
		ROS_INFO_STREAM("Frontier index at end " << _gg.frontiers_counter);
		return _gg.frontiers_counter;
	}
}

int GlobalGraphHandler::availablePathIndex() {
	if (!_available_paths.empty()) {
		ROS_INFO_STREAM("Available path index " << *_available_paths.begin());
		return *_available_paths.begin();
	} else {
		ROS_INFO_STREAM("Path index at end " << _gg.paths_counter);
		return _gg.paths_counter;
	}
}

void GlobalGraphHandler::deactivateFrontiersInLocalGraph(
		rrg_nbv_exploration_msgs::Graph &rrg,
		geometry_msgs::Point robot_position) {
	std::set<int> pruned_frontiers;
	std::set<int> pruned_paths;
	std::vector<std::pair<int, double>> frontiers_in_sensor_range =
			_global_graph_searcher->searchInRadius(robot_position,
					_local_graph_radius_squared); //find all frontiers in sensor range around the robot
	if (frontiers_in_sensor_range.size() > 0)
		ROS_INFO_STREAM("+++++ Deactivate frontiers in local graph");
	for (auto frontier_in_sensor_range : frontiers_in_sensor_range) {
		if (frontier_in_sensor_range.first
				!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN
				&& !_gg.frontiers.at(frontier_in_sensor_range.first).inactive) { //never prune origin and ignore inactive frontiers
			std::vector<std::pair<int, double>> nodes_in_range =
					_graph_searcher->searchInRadius(
							_gg.frontiers.at(frontier_in_sensor_range.first).viewpoint,
							_inflation_active ?
									pow(_robot_radius + rrg.largest_node_radius,
											2) :
									_max_edge_distance_squared);
			for (auto node_in_range : nodes_in_range) {
				double compare_distance =
						_inflation_active ?
								pow(
										sqrt(
												rrg.nodes.at(
														node_in_range.first).squared_radius
														- pow(_robot_width / 2,
																2))
												+ _path_box_distance_thres, 2) :
								_robot_radius_squared;
				if (node_in_range.second <= compare_distance
						|| (!_inflation_active
								&& _collision_checker->checkConnectionToFrontier(
										rrg, node_in_range.first,
										_gg.frontiers.at(
												frontier_in_sensor_range.first).viewpoint,
										sqrt(node_in_range.second)))) { //node overlaps enough with frontier with robot radius
					addFrontierToBePruned(frontier_in_sensor_range.first,
							pruned_frontiers, pruned_paths, rrg);
					break;
				}
			}
		}
	}
	if (pruned_frontiers.size() > 0) {
		handlePrunedFrontiers(pruned_frontiers);
		handlePrunedPaths(pruned_paths);
		ROS_INFO_STREAM("----- Deactivate frontiers in local graph");
	}
}

void GlobalGraphHandler::addFrontierToBePruned(int frontier_to_prune,
		std::set<int> &pruned_frontiers, std::set<int> &pruned_paths,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("Add frontier to be pruned " << frontier_to_prune);
	pruned_frontiers.insert(frontier_to_prune);
	for (auto path : _gg.frontiers.at(frontier_to_prune).paths) {
		pruned_paths.insert(path);
		if (_gg.paths.at(path).connecting_node >= 0) //remove possible connection from node in RRG
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
	while (pruned_frontier != pruned_frontiers.rend()) { //remove or add inactive frontiers
		deactivateFrontier(*pruned_frontier);
		if (*pruned_frontier == _gg.frontiers_counter - 1) {
			int next_pruned_frontier =
					std::next(pruned_frontier) == pruned_frontiers.rend() ?
							(*pruned_frontier - 1) :
							*std::next(pruned_frontier);
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
			ROS_INFO_STREAM("Add available frontier " << *pruned_frontier);
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
		ROS_WARN_STREAM(
				"Removed " << removals << " available frontiers above equal index "<< _gg.frontiers_counter);
	}
}

void GlobalGraphHandler::handlePrunedPaths(const std::set<int> &pruned_paths) {
	ROS_INFO_STREAM("Handle pruned paths, path counter " << _gg.paths_counter);
	bool removed_paths = false;
	auto pruned_path = pruned_paths.rbegin();
	while (pruned_path != pruned_paths.rend()) { //remove or add inactive paths
		deactivatePath(*pruned_path);
		if (*pruned_path == _gg.paths_counter - 1) {
			int next_pruned_path =
					std::next(pruned_path) == pruned_paths.rend() ?
							(*pruned_path - 1) : *std::next(pruned_path);
			for (int i = *pruned_path; i > next_pruned_path; i--) {
				if (_gg.paths.at(i).inactive) {
					ROS_INFO_STREAM(
							"Remove path " << i << " from back of list");
					_gg.paths.pop_back();
					_gg.paths_counter--;
					removed_paths = true;
				} else {
					break;
				}
			}
		} else {
			ROS_INFO_STREAM("Add available path " << *pruned_path);
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
		ROS_WARN_STREAM(
				"Removed " << removals << " available paths above equal index "<< _gg.paths_counter);
	}
}

void GlobalGraphHandler::deactivateFrontier(int pruned_frontier) {
	ROS_INFO_STREAM("Deactivate frontier " << pruned_frontier);
	_gg.frontiers.at(pruned_frontier).inactive = true;
	_gg.frontiers.at(pruned_frontier).paths.clear();
	_gg.frontiers.at(pruned_frontier).paths_counter = 0;
	_gg.frontiers.at(pruned_frontier).viewpoint.x = 0;
	_gg.frontiers.at(pruned_frontier).viewpoint.y = 0;
}

void GlobalGraphHandler::deactivatePath(int pruned_path) {
	ROS_INFO_STREAM("Deactivate path " << pruned_path);
	if (_gg.paths.at(pruned_path).frontier < _gg.frontiers_counter
			&& !_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).inactive) { //remove path from list at frontier
		_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.erase(
				std::remove(
						_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.begin(),
						_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.end(),
						pruned_path),
				_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths.end());
		ROS_INFO_STREAM(
				"Removed path from " << _gg.paths.at(pruned_path).frontier << " path list");
		_gg.frontiers.at(_gg.paths.at(pruned_path).frontier).paths_counter--;

	}
	if (_gg.paths.at(pruned_path).connection
			> rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH
			&& _gg.paths.at(pruned_path).connection < _gg.frontiers_counter
			&& !_gg.frontiers.at(_gg.paths.at(pruned_path).connection).inactive) { //remove path from list at connected frontier
		_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.erase(
				std::remove(
						_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.begin(),
						_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.end(),
						pruned_path),
				_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths.end());
		_gg.frontiers.at(_gg.paths.at(pruned_path).connection).paths_counter--;
		ROS_INFO_STREAM(
				"Removed path from " << _gg.paths.at(pruned_path).connection << " path list");

	}
	_gg.paths.at(pruned_path).inactive = true;
	_gg.paths.at(pruned_path).waypoints.clear();
	_gg.paths.at(pruned_path).length = 0;
}

void GlobalGraphHandler::checkPathsWaypoints(
		rrg_nbv_exploration_msgs::Graph &rrg, int new_node) {
	ROS_INFO_STREAM("+++++ checkPathsWaypoints at node " << new_node);
	for (int edge : rrg.nodes.at(new_node).edges) { //iterate over all neighbors of a new node in the RRG
		int neighbor_node =
				rrg.edges.at(edge).first_node == new_node ?
						rrg.edges.at(edge).second_node :
						rrg.edges.at(edge).first_node;
//		ROS_INFO_STREAM(
//				"Neighbor " << neighbor_node << " has " << rrg.nodes.at(neighbor_node).connected_to.size() << " connections");
		std::vector<int> removed_connections; //paths that have to be removed from neighbor node
		for (int path : rrg.nodes.at(neighbor_node).connected_to) { //iterate over paths connected to this neighbor
			if (_gg.paths.at(path).waypoints.size() > 1) { //only try to prune paths with more than one waypoint
				_global_path_waypoint_searcher->rebuildIndex(
						_gg.paths.at(path));
				std::vector<std::pair<int, double>> waypoints_near_new_node =
						_global_path_waypoint_searcher->searchInRadius(
								rrg.nodes.at(new_node).position,
								_inflation_active ?
										pow(
												_robot_radius
														+ rrg.nodes.at(
																neighbor_node).radius,
												2) :
										_max_edge_distance_squared); //all waypoints that can be connected to the new node
//				ROS_INFO_STREAM(
//						"For path " << path << ", " << waypoints_near_new_node.size() << " waypoints are nearby");
				if (waypoints_near_new_node.size() > 0) {
					int closest_waypoint_to_frontier =
							getClosestWaypointToFrontier(path, new_node,
									waypoints_near_new_node, rrg);
//					ROS_INFO_STREAM(
//							"Closest connectable waypoint: " << closest_waypoint_to_frontier);
					if (closest_waypoint_to_frontier
							< (_gg.paths.at(path).waypoints.size() - 1)) { //if path can be pruned
						double new_path_length = calculateNewPathLength(path,
								new_node, closest_waypoint_to_frontier, rrg);
//						ROS_INFO_STREAM(
//								"Prev path length: " << _gg.paths.at(path).length << " + " <<rrg.nodes.at(_gg.paths.at(path).connecting_node).distance_to_robot << " new "<< new_path_length << " + "<<rrg.nodes.at(new_node).distance_to_robot);
						if ((new_path_length
								+ rrg.nodes.at(new_node).distance_to_robot)
								< (_gg.paths.at(path).length
										+ rrg.nodes.at(
												_gg.paths.at(path).connecting_node).distance_to_robot)) { //only re-route path if length decreases
							ROS_INFO_STREAM(
									"Reduced path "<< path << " from frontier " << _gg.paths.at(path).frontier << " to new node " << new_node << " (prev " << neighbor_node << ") from "<< (_gg.paths.at(path).length+rrg.nodes.at(_gg.paths.at(path).connecting_node).distance_to_robot) << " to " <<(new_path_length + rrg.nodes.at(new_node).distance_to_robot));
							rewirePathToNewNode(path,
									closest_waypoint_to_frontier,
									new_path_length, new_node, rrg);
							removed_connections.push_back(path);
							tryToImproveConnectionsToOtherFrontiers(new_node,
									path, rrg);
							rrg.nodes.at(new_node).connected_to.push_back(path);
						}
					}
				}
			}
		}
		for (int removed_connection : removed_connections) { //remove connected paths from neighbor node that were moved to new node
			rrg.nodes.at(neighbor_node).connected_to.erase(
					std::remove(
							rrg.nodes.at(neighbor_node).connected_to.begin(),
							rrg.nodes.at(neighbor_node).connected_to.end(),
							removed_connection),
					rrg.nodes.at(neighbor_node).connected_to.end());
		}
	}
	ROS_INFO_STREAM("----- checkedPathsWaypoints at node " << new_node);
}

int GlobalGraphHandler::getClosestWaypointToFrontier(int path, int new_node,
		std::vector<std::pair<int, double> > &waypoints_near_new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("getClosestWaypointToFrontier");
	int closest_waypoint_to_frontier = _gg.paths.at(path).waypoints.size() - 1; //index of last waypoint in list
//find connectable waypoint closest to the frontier (most path reduction)
	if (_inflation_active) {
		for (auto waypoint_near_new_node : waypoints_near_new_node) {
			//							ROS_INFO_STREAM(
			//									"Waypoint " << waypoint_near_new_node.first << " at distance " << waypoint_near_new_node.second << " (" << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).x << ", " << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).y << ")");
			closest_waypoint_to_frontier = std::min(
					waypoint_near_new_node.first, closest_waypoint_to_frontier);
		}
	} else {
		std::sort(waypoints_near_new_node.begin(),
				waypoints_near_new_node.end(),
				[](std::pair<int, double> first_waypoint,
						std::pair<int, double> second_waypoint) {
					return first_waypoint.first < second_waypoint.first;
				}
		); //sort waypoints ascending by index
		for (auto waypoint_near_new_node : waypoints_near_new_node) {
			//							ROS_INFO_STREAM(
			//									"Waypoint " << waypoint_near_new_node.first << " at distance " << waypoint_near_new_node.second << " (" << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).x << ", " << _gg.paths.at(path).waypoints.at(waypoint_near_new_node.first).y << ")");
			if (_collision_checker->checkConnectionToFrontier(rrg,
					waypoint_near_new_node.first,
					rrg.nodes.at(new_node).position,
					sqrt(waypoint_near_new_node.second))) {
				//check if a connection can be made
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
	ROS_INFO_STREAM("calculateNewPathLength");
	double path_reduction = 0;
	double distance_to_new_node = sqrt(
			pow(
					_gg.paths.at(path).waypoints.back().x
							- rrg.nodes.at(new_node).position.x, 2)
					+ pow(
							_gg.paths.at(path).waypoints.back().y
									- rrg.nodes.at(new_node).position.y, 2)); //distance between closest waypoint and new connecting node
	for (int i = _gg.paths.at(path).waypoints.size() - 1;
			i > closest_waypoint_to_frontier; i--) { //calculate distances between waypoints to be pruned from path
		path_reduction +=
				sqrt(
						pow(
								_gg.paths.at(path).waypoints.at(i).x
										- _gg.paths.at(path).waypoints.at(i - 1).x,
								2)
								+ pow(
										_gg.paths.at(path).waypoints.at(i).y
												- _gg.paths.at(path).waypoints.at(
														i - 1).y, 2));
	}
	return _gg.paths.at(path).length - path_reduction + distance_to_new_node;
}

void GlobalGraphHandler::rewirePathToNewNode(int path,
		int closest_waypoint_to_frontier, double new_path_length, int new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("rewirePathToNewNode");
	_gg.paths.at(path).waypoints.erase(
			_gg.paths.at(path).waypoints.begin() + closest_waypoint_to_frontier
					+ 1, _gg.paths.at(path).waypoints.end());
	_gg.paths.at(path).length = new_path_length;
	_gg.paths.at(path).waypoints.push_back(rrg.nodes.at(new_node).position);
	_gg.paths.at(path).connecting_node = new_node;
}

void GlobalGraphHandler::tryToImproveConnectionsToOtherFrontiers(int new_node,
		int path, rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("tryToImproveConnectionsToOtherFrontiers");
	int current_frontier = _gg.paths.at(path).frontier; //look for new connections with this frontier
//							ROS_INFO_STREAM(
//									"Try to improve connections from current frontier " << current_frontier << " with path " << path);
	std::set<int> new_frontier_connections; //set of existing connected frontiers to the connecting node which will be reduced by the frontiers already connected to the current frontier

	for (auto connected_path : rrg.nodes.at(new_node).connected_to) {
		new_frontier_connections.insert(_gg.paths.at(connected_path).frontier);
	}
	for (auto frontier_path : _gg.frontiers.at(current_frontier).paths) {
		//								ROS_INFO_STREAM("Path " << frontier_path);
		int other_frontier =
				current_frontier == _gg.paths.at(frontier_path).frontier ?
						_gg.paths.at(frontier_path).connection :
						_gg.paths.at(frontier_path).frontier;
		if (_gg.paths.at(frontier_path).connection
				!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH
				&& new_frontier_connections.erase(other_frontier) > 0) {
			//remove existing connection from new frontier connections (if present, try to improve path)
			int other_path = _gg.frontiers.at(other_frontier).paths.at(0);
			//									ROS_INFO_STREAM(
			//											"Existing path to " << other_frontier << " with length " << _gg.paths.at(frontier_path).length << " compared to length " << _gg.paths.at(path).length + _gg.paths.at(other_path).length);
			if ((_gg.paths.at(path).length + _gg.paths.at(other_path).length)
					< _gg.paths.at(frontier_path).length) {
				//path through new node would be shorter
				ROS_INFO_STREAM(
						"Improve path "<< other_path << " to existing frontier " << other_frontier << " with length " << _gg.paths.at(frontier_path).length << " compared to new length " << _gg.paths.at(path).length + _gg.paths.at(other_path).length);
				improvePathToConnectedFrontier(frontier_path, path,
						other_frontier, other_path);
			}
		}
	}
//							ROS_INFO_STREAM(
//									"Add new " << new_frontier_connections.size()<<" connections");
	for (auto new_frontier_connection : new_frontier_connections) {
		connectFrontiers(current_frontier, new_frontier_connection, path,
				_gg.frontiers.at(new_frontier_connection).paths.at(0), true); //first path of frontier is always path to local graph
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
			_gg.paths.at(path).waypoints.end() - 1); //omit last element to avoid duplicate waypoint
	_gg.paths.at(frontier_path).waypoints.insert(
			_gg.paths.at(frontier_path).waypoints.end(),
			_gg.paths.at(other_path).waypoints.rbegin(),
			_gg.paths.at(other_path).waypoints.rend());
}

bool GlobalGraphHandler::calculateNextFrontierGoal(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	ROS_INFO_STREAM("+++++ Calculate Next Frontier Goal");
	int best_frontier = -1;
	int best_path = -1;
	std::vector<int> active_frontiers;
	for (auto &frontier : _gg.frontiers) {
		if (!frontier.inactive)
			active_frontiers.push_back(frontier.index);
	}
	if (active_frontiers.size() == 1 && _auto_homing) { //only origin remains
		ROS_INFO_STREAM("only one frontier remaining->origin");
		best_frontier = 0;
		best_path = 0;
	} else if (active_frontiers.size() <= 1) {
		ROS_INFO_STREAM("----- Calculate Next Frontier Goal, no frontier");
		return false;
	} else {
		ROS_INFO_STREAM(
				"Connect all frontiers with nearest node " << rrg.nearest_node << " and each other");
		for (auto &frontier : _gg.frontiers) { //establish all missing connection between frontiers through the RRG
			if (!frontier.inactive
					&& frontier.paths_counter < active_frontiers.size()) { //one path to every other frontier + 1 to RRG required
				std::set<int> missing_frontiers(active_frontiers.begin(),
						active_frontiers.end());
				missing_frontiers.erase(frontier.index);
				for (int path : frontier.paths) {
					if (_gg.paths.at(path).connection
							!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH)
						missing_frontiers.erase(
								frontier.index == _gg.paths.at(path).frontier ?
										_gg.paths.at(path).connection :
										_gg.paths.at(path).frontier);
				}
				for (auto missing_frontier : missing_frontiers) { //build the missing paths
					if (frontier.index > missing_frontier) { //only build paths where this frontier will be frontier in path
						ROS_INFO_STREAM(
								"Establish missing connection between " << frontier.index <<" and " << missing_frontier);
						int frontier_connecting_node = _gg.paths.at(
								frontier.paths.front()).connecting_node;
						int missing_frontier_connecting_node =
								_gg.paths.at(
										_gg.frontiers.at(missing_frontier).paths.front()).connecting_node;
						int first_mutual_node_in_path = rrg.nearest_node;
						double minimum_local_path_length =
								rrg.nodes.at(frontier_connecting_node).distance_to_robot
										+ rrg.nodes.at(
												missing_frontier_connecting_node).distance_to_robot;
						ROS_INFO_STREAM(
								"Frontier connecting node " << frontier_connecting_node << " missing frontier connecting node " << missing_frontier_connecting_node << " min path length " << minimum_local_path_length);
						std::vector<int> shortest_local_path; //the remaining path between the frontiers in the local graph
						for (auto rit =
								rrg.nodes.at(frontier_connecting_node).path_to_robot.rbegin();
								rit
										!= std::prev(
												rrg.nodes.at(
														frontier_connecting_node).path_to_robot.rend()); //reverse end is connecting node
								++rit) { //find mutual node in path and length of the local path through this node
							ROS_INFO_STREAM(
									"Check if node " << *rit << " from frontier's path to robot is also present in missing frontier's path to robot");
							auto it =
									std::find(
											std::next(
													rrg.nodes.at(
															missing_frontier_connecting_node).path_to_robot.begin()), //first node is connecting node
											rrg.nodes.at(
													missing_frontier_connecting_node).path_to_robot.end(),
											*rit); //find occurence of node in both paths
							if (it
									!= rrg.nodes.at(
											missing_frontier_connecting_node).path_to_robot.end()) { //found mutual node in paths
								ROS_INFO_STREAM(
										"First mutual node in local RRG is " << *it);
								first_mutual_node_in_path = *it;
								shortest_local_path.insert(
										shortest_local_path.end(),
										rrg.nodes.at(frontier_connecting_node).path_to_robot.rbegin(),
										rit);
								shortest_local_path.insert(
										shortest_local_path.end(),
										std::next(it), //skip it, already present in path
										rrg.nodes.at(
												missing_frontier_connecting_node).path_to_robot.end());
								std::string rempa = "";
								for (int rem : shortest_local_path) {
									rempa += std::to_string(rem) + ",";
								}
								ROS_INFO_STREAM(
										"Remaining path in local RRG is: " << rempa);
								double length = 0;
								for (int i = 0;
										i < shortest_local_path.size() - 1;
										i++) { //iterate over edges in remaining path to calculate length
									int edge =
											_graph_path_calculator->findExistingEdge(
													rrg,
													shortest_local_path.at(i),
													shortest_local_path.at(
															i + 1));
									if (edge != -1) {
										length += rrg.edges.at(edge).length;
									} else {
										ROS_WARN_STREAM(
												"Unable to calculate minimum local path, node " << shortest_local_path.at(i) << " has no edge to " << shortest_local_path.at(i+1));
										break;
									}
								}
								if (length > 0) {
									minimum_local_path_length = length;
									ROS_INFO_STREAM(
											"Minimum local path length at mutual node "<< *it << " is " << minimum_local_path_length);
									break;
								}
								ROS_INFO_STREAM(
										"Continue looking for mutual node");
							}
						}
						if (shortest_local_path.empty()) {
							ROS_INFO_STREAM(
									"No mutual node, use full paths to robot");
							shortest_local_path.insert(
									shortest_local_path.end(),
									rrg.nodes.at(frontier_connecting_node).path_to_robot.rbegin(),
									rrg.nodes.at(frontier_connecting_node).path_to_robot.rend());
							shortest_local_path.insert(
									shortest_local_path.end(),
									std::next(
											rrg.nodes.at(
													frontier_connecting_node).path_to_robot.begin()), //skip it, already present in path
									rrg.nodes.at(
											missing_frontier_connecting_node).path_to_robot.end());
						}
						std::vector<int> path;
						double distance = 0;
						_graph_path_calculator->findShortestRoutes(rrg,
								frontier_connecting_node,
								missing_frontier_connecting_node, path,
								minimum_local_path_length); //add verbose output
						if (!path.empty())
							shortest_local_path = path;
						//build path
						ROS_INFO_STREAM(
								"Build path between " << frontier.index << " and " <<missing_frontier);
						rrg_nbv_exploration_msgs::GlobalPath path_between_frontiers;
						path_between_frontiers.index = availablePathIndex();
						path_between_frontiers.frontier = frontier.index;
						path_between_frontiers.connection = missing_frontier;
						path_between_frontiers.waypoints.insert(
								path_between_frontiers.waypoints.begin(),
								_gg.paths.at(
										_gg.frontiers.at(frontier.index).paths.front()).waypoints.begin(),
								_gg.paths.at(
										_gg.frontiers.at(frontier.index).paths.front()).waypoints.end());
						for (int i = 1; i < shortest_local_path.size() - 1;
								i++) { //omit begin and end, connecting nodes are already present in path
							path_between_frontiers.waypoints.push_back(
									rrg.nodes.at(shortest_local_path.at(i)).position);
						}
						path_between_frontiers.waypoints.insert(
								path_between_frontiers.waypoints.end(),
								_gg.paths.at(
										_gg.frontiers.at(missing_frontier).paths.front()).waypoints.rbegin(),
								_gg.paths.at(
										_gg.frontiers.at(missing_frontier).paths.front()).waypoints.rend());

						path_between_frontiers.length =
								_gg.paths.at(
										_gg.frontiers.at(frontier.index).paths.front()).length
										+ minimum_local_path_length
										+ _gg.paths.at(
												_gg.frontiers.at(
														missing_frontier).paths.front()).length;
						path_between_frontiers.connecting_node = -1;
						_gg.frontiers.at(frontier.index).paths.push_back(
								path_between_frontiers.index);
						_gg.frontiers.at(frontier.index).paths_counter++;
						_gg.frontiers.at(missing_frontier).paths.push_back(
								path_between_frontiers.index);
						_gg.frontiers.at(missing_frontier).paths_counter++;
						insertPathInGg(path_between_frontiers);
						ROS_INFO_STREAM(
								"Established path " <<path_between_frontiers.index << " between frontiers " << path_between_frontiers.frontier << " and " << path_between_frontiers.connection << " with length " << path_between_frontiers.length);
					}
				}
			}
		}
		ROS_INFO_STREAM("established missing connections");

		for (auto &path : _gg.paths) { //connect all frontiers directly to the nearest node to the robot
			if (!path.inactive
					&& path.connection
							== rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) { //add path to robot from connecting node
				for (int i =
						rrg.nodes.at(path.connecting_node).path_to_robot.size()
								- 2; i >= 0; i--) { //skip node itself, as it already is in waypoints
					path.waypoints.push_back(
							rrg.nodes.at(
									rrg.nodes.at(path.connecting_node).path_to_robot.at(
											i)).position);
				}
				path.length +=
						rrg.nodes.at(path.connecting_node).distance_to_robot;
				path.connecting_node = rrg.nearest_node;
			}
		}
		ROS_INFO_STREAM(
				"Connected all frontiers directly to nearest node " << rrg.nearest_node);

		//TODO: call TSP solver to get next frontier goal

		double distance_to_frontier = std::numeric_limits<double>::infinity();
		for (auto frontier : _gg.frontiers) {
			if (frontier.index
					!= rrg_nbv_exploration_msgs::GlobalPath::ORIGIN) { //origin is always last goal
				if (_gg.paths.at(frontier.paths.front()).length
						< distance_to_frontier) {
					best_frontier = frontier.index;
					best_path = frontier.paths.front();
					distance_to_frontier =
							_gg.paths.at(frontier.paths.front()).length;
				}
			}
		}
		ROS_INFO_STREAM(
				"Next frontier goal is " << best_frontier << " with path " << best_path);

	}
	_next_frontier_with_path = std::make_pair(best_frontier, best_path);
	_global_path_waypoint_searcher->rebuildIndex(_gg.paths.at(best_path));
	_active_paths_closest_waypoint = std::make_pair(best_path,
			_gg.paths.at(best_path).waypoints.size() - 1); //start at waypoint at nearest node in RRG
	ROS_INFO_STREAM("----- Calculate Next Frontier Goal");
	return true;
}

bool GlobalGraphHandler::checkIfNextFrontierWithPathIsValid() {
	return _next_frontier_with_path.first >= 0
			&& _next_frontier_with_path.first < _gg.frontiers_counter
			&& _next_frontier_with_path.second >= 0
			&& _next_frontier_with_path.second < _gg.paths_counter;
}

bool GlobalGraphHandler::getFrontierGoal(geometry_msgs::Point &goal,
		double &yaw, geometry_msgs::Point &robot_pos) {
	if (checkIfNextFrontierWithPathIsValid()) {
		goal = _gg.frontiers.at(_next_frontier_with_path.first).viewpoint;
		if (_gg.paths.at(_next_frontier_with_path.second).waypoints.size()
				> 1) {
//			int waypoint_index =
//					_gg.paths.at(_next_frontier_with_path.second).frontier
//							== _next_frontier_with_path.first ?
//							1 :
//							*std::prev(
//									_gg.paths.at(
//											_next_frontier_with_path.second).waypoints.end(),
//									2); //if frontier goal is frontier in path, second waypoint index is relevant for yaw, otherwise second to last (frontier is always first waypoint, connection last)
			yaw =
					(int) (atan2(
							goal.y
									- _gg.paths.at(
											_next_frontier_with_path.second).waypoints.at(
											1).y,
							goal.x
									- _gg.paths.at(
											_next_frontier_with_path.second).waypoints.at(
											1).x) * 180 / M_PI);
		} else { //frontier directly next to robot, define yaw from robot position
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
		ROS_INFO_STREAM("Get frontier path");
//		if (_gg.paths.at(_next_frontier_with_path.second).frontier
//				== _next_frontier_with_path.first) {
		std::vector<geometry_msgs::Point> waypoints;
		int connecting_node;
		waypoints.insert(waypoints.end(),
				_gg.paths.at(_next_frontier_with_path.second).waypoints.begin(),
				_gg.paths.at(_next_frontier_with_path.second).waypoints.end());
//		} else { //reverse order of waypoints (frontier is always first waypoint, connection last)
//			waypoints.insert(waypoints.end(),
//					_gg.paths.at(_next_frontier_with_path.second).waypoints.rbegin(),
//					_gg.paths.at(_next_frontier_with_path.second).waypoints.rend());
//
//		}
		connecting_node =
				_gg.paths.at(_next_frontier_with_path.second).connecting_node;
		_graph_path_calculator->getNavigationPath(path, waypoints, robot_pos,
				_active_paths_closest_waypoint.second);
		return true;
	} else {
		ROS_INFO_STREAM(
				"Unable to retrieve frontier path because it is not in the list");
		return false;
	}
}

std::vector<int> GlobalGraphHandler::frontierReached() {
	ROS_INFO_STREAM("+++++ frontierReached");
	std::vector<int> connected_paths;
	if (checkIfNextFrontierWithPathIsValid()) {
		int frontier = _next_frontier_with_path.first;
		ROS_INFO_STREAM("Frontier " << frontier);
		_next_frontier_with_path = std::make_pair(-1, -1);
		std::set<int> pruned_frontiers;
		std::set<int> pruned_paths;
		pruned_frontiers.insert(frontier);
		for (auto path : _gg.frontiers.at(frontier).paths) {
			pruned_paths.insert(path);
			if (_gg.paths.at(path).connection
					!= rrg_nbv_exploration_msgs::GlobalPath::LOCAL_GRAPH) {
				int other_frontier =
						_gg.paths.at(path).frontier == frontier ?
								_gg.paths.at(path).connection :
								_gg.paths.at(path).frontier;
				ROS_INFO_STREAM(
						"Overwrite path to local graph of frontier " << other_frontier << " with path " << path);
				//overwrite path between other frontier and local graph with details from the path between those two
				_gg.paths.at(_gg.frontiers.at(other_frontier).paths.front()).connecting_node =
						0; //root at new RRG
				_gg.paths.at(_gg.frontiers.at(other_frontier).paths.front()).waypoints.clear();
				if (other_frontier > frontier) {
					_gg.paths.at(_gg.frontiers.at(other_frontier).paths.front()).waypoints.insert(
							_gg.paths.at(
									_gg.frontiers.at(other_frontier).paths.front()).waypoints.end(),
							_gg.paths.at(path).waypoints.begin(),
							_gg.paths.at(path).waypoints.end());
				} else { //path was from frontier to other frontier, must be reversed
					_gg.paths.at(_gg.frontiers.at(other_frontier).paths.front()).waypoints.insert(
							_gg.paths.at(
									_gg.frontiers.at(other_frontier).paths.front()).waypoints.end(),
							_gg.paths.at(path).waypoints.rbegin(),
							_gg.paths.at(path).waypoints.rend());
				}
				_gg.paths.at(_gg.frontiers.at(other_frontier).paths.front()).length =
						_gg.paths.at(path).length;
				_gg.paths.at(path).connecting_node = -1; //prevents erase at connecting node when pruning frontier
				connected_paths.push_back(
						_gg.frontiers.at(other_frontier).paths.front());
			}
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

bool GlobalGraphHandler::updateClosestWaypoint(
		geometry_msgs::Point &robot_pos) {
	double min_distance;
	int nearest_node;
	_global_path_waypoint_searcher->findNearestNeighbour(robot_pos,
			min_distance, nearest_node);
	if (nearest_node != _active_paths_closest_waypoint.second) {
		_active_paths_closest_waypoint.second = nearest_node;
		ROS_INFO_STREAM("New closest waypoint is " << nearest_node);
		if (nearest_node == 0) { //frontier reached
			return true;
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
