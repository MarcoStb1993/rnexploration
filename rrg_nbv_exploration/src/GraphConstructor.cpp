#include <rrg_nbv_exploration/GraphConstructor.h>

namespace rrg_nbv_exploration {
GraphConstructor::GraphConstructor() {
}

GraphConstructor::~GraphConstructor() {
}

void GraphConstructor::initialization(geometry_msgs::Point seed) {
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_max_range", _sensor_range, 5.0);
	_radius_search_range = pow(2 * _sensor_range, 2);
	private_nh.param("local_graph_radius", _local_graph_radius, 5.0);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	double min_edge_distance;
	private_nh.param("min_edge_distance", _min_edge_distance, 1.0);
	private_nh.param("max_edge_distance", _max_edge_distance, 2.0);
	_max_edge_distance_squared = pow(_max_edge_distance, 2);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	_robot_radius_squared = pow(_robot_radius, 2);
	private_nh.param("grid_map_resolution", _grid_map_resolution, 0.05);
	private_nh.param("local_sampling_radius", _local_sampling_radius, 5.0);
	private_nh.param("local_exploration_finished_timer_duration",
			_local_exploration_finished_timer_duration, 10.0);
	private_nh.param("max_consecutive_failed_goals",
			_max_consecutive_failed_goals, 5);
	private_nh.param("reupdate_nodes", _reupdate_nodes, true);
	private_nh.param("samples_per_loop", _samples_per_loop, 10);

	ros::NodeHandle nh("rne");
	_rrg_publisher = nh.advertise<rrg_nbv_exploration_msgs::Graph>("rrg", 1);
	_node_to_update_publisher = nh.advertise<
			rrg_nbv_exploration_msgs::NodeToUpdate>("node_to_update", 1);
	_updated_node_subscriber = nh.subscribe("updated_node", 1,
			&GraphConstructor::updatedNodeCallback, this);
	_exploration_goal_obsolete_publisher = nh.advertise<
			rrg_nbv_exploration_msgs::ExplorationGoalObsolete>(
			"explorationGoalObsolete", 1);
	_request_goal_service = nh.advertiseService("requestGoal",
			&GraphConstructor::requestGoal, this);
	_request_path_service = nh.advertiseService("requestPath",
			&GraphConstructor::requestPath, this);
	_update_current_goal_service = nh.advertiseService("updateCurrentGoal",
			&GraphConstructor::updateCurrentGoal, this);

	_local_exploration_finished_timer = _nh.createTimer(
			ros::Duration(_local_exploration_finished_timer_duration),
			&GraphConstructor::localExplorationFinishedTimerCallback, this,
			false, false);

	_set_rrg_state_service = nh.advertiseService("setRrgState",
			&GraphConstructor::setRrgState, this);
	_get_rrg_state_service = nh.advertiseService("getRrgState",
			&GraphConstructor::getRrgState, this);
	_reset_rrg_state_service = nh.advertiseService("resetRrgState",
			&GraphConstructor::resetRrgState, this);

	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&GraphConstructor::convertOctomapMsgToOctree, this);

	_local_graph_radius = std::max(_local_graph_radius, 2 * _robot_radius);	// cannot fall below the robot diameter
	_local_graph_radius_squared = pow(_local_graph_radius + _robot_radius, 2);// add buffer to pruning
	_local_sampling_radius = std::min(_local_graph_radius,
			_local_sampling_radius); // cannot exceed local graph radius

	_graph_searcher.reset(new GraphSearcher());
	_collision_checker.reset(new CollisionChecker());
	_graph_path_calculator.reset(new GraphPathCalculator());
	_node_comparator.reset(new NodeComparator());
	_global_graph_handler.reset(new GlobalGraphHandler());
	_running = false;
	_local_running = false;
	_local_sampling_radius = std::min(_local_sampling_radius,
			_local_graph_radius); // cannot sample outside local area
}

void GraphConstructor::initExploration(const geometry_msgs::Point &seed) {
	_pursuing_global_goal = false;
	_reached_frontier_goal = false;
	_global_goal_updated = true;
	_updating_global_goal = false;
	initLocalGraph(seed);
	resetHelperClasses();
	_global_graph_handler->initialize(_rrg.nodes[0], _graph_path_calculator,
			_graph_searcher, _collision_checker);
	_generator.seed(time(NULL));
}

void GraphConstructor::initLocalGraph(
		const geometry_msgs::Point &root_position) {
	_nodes_to_update.clear();
	_nodes_to_reupdate.clear();
	_last_three_nodes_path.clear();
	_new_node_positions.clear();
	_rrg.header.frame_id = "/map";
	_rrg.ns = "rrg";
	_rrg.nodes.clear();
	_rrg.edges.clear();
	_rrg.node_counter = 0;
	_rrg.edge_counter = 0;
	rrg_nbv_exploration_msgs::Node root;
	root.position = root_position;
	root.position.z += _sensor_height;
	root.status = rrg_nbv_exploration_msgs::Node::VISITED;
	root.gain = -1;
	root.reward_function = 0;
	root.index = 0;
	_rrg.nodes.push_back(root);
	_rrg.node_counter++;
	_rrg.nearest_node = 0;
	_current_goal_node = -1;
	_last_goal_node = -1;
	_last_updated_node = -1;
	_local_goal_updated = true;
	_updating_local_goal = false;
	_sort_nodes_to_update = false;
	_goal_obsolete = false;
	_pursued_local_goal = false;
	_local_goal_obsolete = false;
	_consecutive_failed_goals = 0;
	_last_robot_pos = root_position;
	_nodes_to_update.push_back(0);
	_last_three_nodes_path.push_back(0);
}

void GraphConstructor::resetHelperClasses() {
	_graph_searcher->initialize(_rrg);
	_node_comparator->initialization();
	_collision_checker->initialize(_rrg, _graph_searcher,
			_graph_path_calculator);
}

void GraphConstructor::runExploration() {
	_rrg.header.stamp = ros::Time::now();
	if (_running) {
		_robot_pose = _graph_path_calculator->getRobotPose();
		if (_local_running && _map_min_bounding[0] && _map_min_bounding[1]
				&& _map_min_bounding[2]) {
			ROS_INFO_STREAM("+++++ runExploration");
			bool new_nearest_node = determineNearestNodeToRobot(
					_robot_pose.position); // check if nearest node to robot changed which means robot moved
			expandGraph(!new_nearest_node);
			if (new_nearest_node) {					// robot moved, update paths
				pruneLocalGraph(); // remove nodes from local RRG that are too far away
				_graph_path_calculator->updatePathsToRobot(_rrg.nearest_node,
						_rrg, _robot_pose, true, _nodes_to_update,
						_sort_nodes_to_update);
				determineNextNodeInPath();
				ROS_INFO_STREAM("determinedNextNodeInPath");
			} else { // check if robot heading changed and update heading
				if (_graph_path_calculator->updateHeadingToRobot(
						_rrg.nearest_node, _rrg, _robot_pose, _next_node,
						_edge_to_next_node, _distance_to_nearest_node_squared,
						_nodes_to_update, _sort_nodes_to_update))
					_node_comparator->robotMoved();
			}
			_node_comparator->maintainList(_rrg);
			checkCurrentGoal();
			publishNodeToUpdate();
			if (_nodes_to_update.empty() && _current_goal_node == -1) {
				_local_exploration_finished_timer.start();
			}
			ROS_INFO_STREAM("----- runExploration");
		} else if (!_local_running) {
			if (!_global_graph_handler->checkIfNextFrontierWithPathIsValid()) {
				if (!_global_graph_handler->calculateNextFrontierGoal(_rrg)) { // exploration finished
					ROS_INFO_STREAM("Exploration finished");
					_running = false;
				} else {
					_global_goal_updated = true;
					_updating_global_goal = false;
				}
			} else if (_global_graph_handler->updateClosestWaypoint(
					_robot_pose.position)) { // frontier reached
				geometry_msgs::Point frontier;
				std::vector<int> connected_paths =
						_global_graph_handler->frontierReached(frontier);
				initLocalGraph(frontier);
				resetHelperClasses();
				_rrg.nodes.front().connected_to = connected_paths;
				_local_running = true;
			}
		}
		publishExplorationGoalObsolete();
	}
	_rrg_publisher.publish(_rrg);
	_global_graph_handler->publishGlobalGraph();
}

void GraphConstructor::expandGraph(bool update_paths) {
	for (int i = 0; i < _samples_per_loop; i++) {
		geometry_msgs::Point rand_sample;
		insertNewNode(samplePoint(rand_sample), rand_sample, update_paths,
				false);
		if (_local_sampling_radius > 0)
			insertNewNode(samplePointLocally(rand_sample), rand_sample,
					update_paths, false);
	}
	for (auto new_node_position : _new_node_positions) { // add stored positions
		ROS_INFO_STREAM("***** Try to add new node at a frontier position");
		insertNewNode(true, new_node_position, update_paths, true);
	}
	_new_node_positions.clear();
}

void GraphConstructor::insertNewNode(bool sampling_success,
		const geometry_msgs::Point &rand_sample, bool update_paths,
		bool unmovable_point) {
	rrg_nbv_exploration_msgs::Node node;
	if (sampling_success
			&& _collision_checker->steer(_rrg, node, rand_sample, _robot_pose,
					unmovable_point)) {
		if (!std::isinf(node.cost_function)) { // do not update unreachable nodes
			if (update_paths)
				_graph_path_calculator->updatePathsToRobot(node.index, _rrg,
						_robot_pose, false, _nodes_to_update,
						_sort_nodes_to_update); // check if new connection could improve other distances
			_nodes_to_update.push_back(node.index);
			_sort_nodes_to_update = true;
		} else {
			ROS_INFO_STREAM("Do not update unreachable node " << node.index);
		}
		_graph_searcher->rebuildIndex(_rrg);
//		_local_exploration_finished_timer.stop();
	}
}

bool GraphConstructor::samplePoint(geometry_msgs::Point &rand_sample) {
	return samplePointInRadius(rand_sample, _robot_pose.position,
			std::min(_sensor_range, _local_graph_radius));
}

bool GraphConstructor::samplePointLocally(geometry_msgs::Point &rand_sample) {
	return samplePointInRadius(rand_sample, _robot_pose.position,
			std::min(_local_sampling_radius, _local_graph_radius));
}

bool GraphConstructor::samplePointInRadius(geometry_msgs::Point &rand_sample,
		geometry_msgs::Point center, double radius) {
	std::uniform_real_distribution<double> r_distribution(0, 1);
	std::uniform_real_distribution<double> theta_distribution(0, 2 * M_PI);
	double r = radius * sqrt(r_distribution(_generator));
	double theta = theta_distribution(_generator);
	rand_sample.x = center.x + r * cos(theta);
	rand_sample.y = center.y + r * sin(theta);
	return (rand_sample.x >= _map_min_bounding[0]
			&& rand_sample.x <= _map_max_bounding[0]
			&& rand_sample.y >= _map_min_bounding[1]
			&& rand_sample.y <= _map_max_bounding[1]);
}

void GraphConstructor::pruneLocalGraph() {
	ROS_INFO_STREAM("+++++ Prune local graph");
	std::set<int> pruned_nodes;
	std::set<int> pruned_edges;
	std::vector<int> disconnected_nodes = findOutOfSensorRadiusNodes(
			pruned_nodes, pruned_edges);
	std::set<int> connected_nodes; // nodes with a path to the robot and an edge to a disconnected node
	findAllConnectedAndDisconnectedNodes(disconnected_nodes, connected_nodes);
	for (auto node : connected_nodes) { // start path update from each connected nodes
		_graph_path_calculator->updatePathsToRobot(node, _rrg, _robot_pose,
				false, _nodes_to_update, _sort_nodes_to_update);
	}
	for (auto node : disconnected_nodes) {
		if (std::isinf(_rrg.nodes[node].cost_function)) { // check if disconnected nodes are still disconnected and deactivate them
														  //				ROS_INFO_STREAM(
														  //						"Node " << node << " terminally disconnected, to be pruned");
			pruned_nodes.insert(node);
			_rrg.nodes[node].status = rrg_nbv_exploration_msgs::Node::INACTIVE;
			for (int edge : _rrg.nodes[node].edges) {
				//					ROS_INFO_STREAM("Edge " << edge << " to be pruned");
				_rrg.edges[edge].inactive = true;
				pruned_edges.insert(edge);
			}
		}
	}
	if (pruned_nodes.size() > 0) {
		handlePrunedNodes(pruned_nodes);
		handlePrunedEdges(pruned_edges);
		_graph_searcher->rebuildIndex(_rrg);
	}
	ROS_INFO_STREAM("----- Pruned local graph");
}

std::vector<int> GraphConstructor::findOutOfSensorRadiusNodes(
		std::set<int> &pruned_nodes, std::set<int> &pruned_edges) {
	ROS_INFO_STREAM("findOutOfSensorRadiusNodes");
	std::set<int> disconnected_nodes_set;
	for (auto &node : _rrg.nodes) {
		if (node.status != rrg_nbv_exploration_msgs::Node::INACTIVE) {
			double distance_squared = pow(
					_robot_pose.position.x - node.position.x, 2)
					+ pow(_robot_pose.position.y - node.position.y, 2);
			if (distance_squared > _local_graph_radius_squared) { // remove node from local graph
				pruned_nodes.insert(node.index);
				_rrg.nodes[node.index].status =
						rrg_nbv_exploration_msgs::Node::INACTIVE;
				disconnected_nodes_set.erase(node.index); // remove in case it was already added
				//				ROS_INFO_STREAM("Node " << node.index << " out of range");
				for (int edge : node.edges) {
					_rrg.edges[edge].inactive = true;
					pruned_edges.insert(edge);
					int remaining_node =
							_rrg.edges[edge].first_node == node.index ?
									_rrg.edges[edge].second_node :
									_rrg.edges[edge].first_node;
					//					ROS_INFO_STREAM(
					//							"Edge " << edge << " to node " << remaining_node << " to be pruned");
					if (isNextInPathToRobot(remaining_node, node.index)) {
						_rrg.nodes[remaining_node].cost_function =
								std::numeric_limits<double>::infinity();
						//						ROS_INFO_STREAM(
						//								"Node "<< remaining_node << " disconnected");
						disconnected_nodes_set.insert(remaining_node);
					}
				}
			}
		}
	}
	std::vector<int> disconnected_nodes(disconnected_nodes_set.begin(),
			disconnected_nodes_set.end()); // convert set to vector for iteration over growing vector
	return disconnected_nodes;
}

bool GraphConstructor::isNextInPathToRobot(int neighbor_node, int node) {
	return _rrg.nodes[neighbor_node].path_to_robot.size() > 1
			&& _rrg.nodes[neighbor_node].path_to_robot.at(
					_rrg.nodes[neighbor_node].path_to_robot.size() - 2) == node;
}

void GraphConstructor::handlePrunedEdges(const std::set<int> &pruned_edges) {
//	ROS_INFO_STREAM("handlePrunedEdges, edge counter " << _rrg.edge_counter);
	bool removed_edges = false;
	auto pruned_edge = pruned_edges.rbegin();
	while (pruned_edge != pruned_edges.rend()) { // remove or add inactive edges
		if (*pruned_edge == _rrg.edge_counter - 1) {
			int next_pruned_edge =
					std::next(pruned_edge) == pruned_edges.rend() ?
							-1 : *std::next(pruned_edge); // if last pruned edge, try to remove inactive edges up to index 0
			for (int i = *pruned_edge; i > next_pruned_edge; i--) {
				if (_rrg.edges.at(i).inactive) {
					ROS_INFO_STREAM(
							"Remove edge " << i << " from back of list");
					_rrg.edges.pop_back();
					_rrg.edge_counter--;
					removed_edges = true;
				} else {
					break;
				}
			}
		} else {
			ROS_INFO_STREAM("Add available edge " << *pruned_edge);
			_collision_checker->addAvailableEdge(*pruned_edge);
		}
		pruned_edge++;
	}
	if (removed_edges) {
		_collision_checker->removeDeletedAvailableEdges(_rrg.edge_counter);
	}
}

void GraphConstructor::removeNodeFromUpdateLists(int node) {
	_nodes_to_update.remove(node);
	_nodes_to_reupdate.erase(
			std::remove(_nodes_to_reupdate.begin(), _nodes_to_reupdate.end(),
					node), _nodes_to_reupdate.end());
	_node_comparator->removeNode(node);
}

void GraphConstructor::handlePrunedNodes(const std::set<int> &pruned_nodes) {
	ROS_INFO_STREAM("handlePrunedNodes, node counter " << _rrg.node_counter);
	std::vector<int> nodes(pruned_nodes.begin(), pruned_nodes.end());
	std::sort(nodes.begin(), nodes.end(), [this](int node_one, int node_two) {
		return sortByPathLength(node_one, node_two);
	}); // sort ascending by path to robot length

	findConnectedNodesWithGain(nodes);
	for (int i = nodes.size() - 1; i >= 0; i--) { // deactivate nodes by path length to merge and continue paths correctly
		deactivateNode(nodes.at(i));
	}
	bool removed_nodes = false;
	auto pruned_node = pruned_nodes.rbegin();
	while (pruned_node != pruned_nodes.rend()) { // remove or add inactive edges
		if (*pruned_node == _rrg.node_counter - 1) {
			int next_pruned_node =
					std::next(pruned_node) == pruned_nodes.rend() ?
							-1 : *std::next(pruned_node); // if last pruned node, try to remove inactive nodes up to root
			for (int i = *pruned_node; i > next_pruned_node; i--) {
				if (_rrg.nodes.at(i).status
						== rrg_nbv_exploration_msgs::Node::INACTIVE) {
					ROS_INFO_STREAM(
							"Remove node " << i << " from back of list");
					_rrg.nodes.pop_back();
					_rrg.node_counter--;
					removed_nodes = true;
				} else {
					break;
				}
			}
		} else {
			ROS_INFO_STREAM("Add available node " << *pruned_node);
			_collision_checker->addAvailableNode(*pruned_node);
		}
		pruned_node++;
	}
	if (removed_nodes) {
		_collision_checker->removeDeletedAvailableNodes(_rrg.node_counter);
	}
}

bool GraphConstructor::sortByPathLength(int node_one, int node_two) {
	return _rrg.nodes[node_one].path_to_robot.size()
			< _rrg.nodes[node_two].path_to_robot.size();
}

void GraphConstructor::findConnectedNodesWithGain(std::vector<int> &nodes) {
	ROS_INFO_STREAM("##### Find connected nodes with gain");
	std::set<int> possible_frontiers;
	for (auto node : nodes) {
		ROS_INFO_STREAM(
				"Pruned node " << node << " with gain " << _rrg.nodes.at(node).gain << " and status " << (int)_rrg.nodes.at(node).status);
		if (_rrg.nodes.at(node).gain > 0.0) {
			possible_frontiers.insert(node);
		}
	}
	ROS_INFO_STREAM(
			"Possible frontiers size " << possible_frontiers.size() << "");
	while (!possible_frontiers.empty()) {
		std::vector<int> frontier;
		frontier.push_back(*possible_frontiers.begin());
		possible_frontiers.erase(possible_frontiers.begin());
		for (int i = 0; i < frontier.size(); i++) { // build frontier from possible frontier nodes
			ROS_INFO_STREAM(
					"Look at node " << frontier.at(i) << " in frontier with size " << frontier.size());
			for (auto edge : _rrg.nodes.at(frontier.at(i)).edges) {
				int neighbor_node =
						frontier.at(i) == _rrg.edges.at(edge).first_node ?
								_rrg.edges.at(edge).second_node :
								_rrg.edges.at(edge).first_node;
				if (_rrg.nodes.at(neighbor_node).gain > 0
						&& possible_frontiers.erase(neighbor_node) > 0) { // neighbor will also become a frontier
					ROS_INFO_STREAM(
							"Add node " << neighbor_node << " to frontier");
					frontier.push_back(neighbor_node);
				}
			}
		}
		int closest_node_to_robot = frontier.front();
		double shortest_distance_to_robot =
				_rrg.nodes.at(closest_node_to_robot).distance_to_robot;
		for (int i = 1; i < frontier.size(); i++) {
			if (_rrg.nodes.at(frontier.at(i)).distance_to_robot
					< shortest_distance_to_robot) {
				_rrg.nodes.at(closest_node_to_robot).gain = 0;
				closest_node_to_robot = frontier.at(i);
				shortest_distance_to_robot =
						_rrg.nodes.at(frontier.at(i)).distance_to_robot;
			} else {
				_rrg.nodes.at(frontier.at(i)).gain = 0;
			}
		}
		ROS_INFO_STREAM(
				"Remaining node from frontier is " << closest_node_to_robot << " with distance " << shortest_distance_to_robot);
	}
}

void GraphConstructor::deactivateNode(int node) {
	// ROS_INFO_STREAM("deactivateNode " << node);
	if (node == _current_goal_node) {
		_local_goal_obsolete = true;
	}
	if (node == _last_updated_node) {
		_last_updated_node = -1;
	}
	if (node == _last_goal_node) {
		_last_goal_node = -1;
	}
	removeNodeFromUpdateLists(node);
	if (_rrg.nodes[node].gain > 0) {
		_global_graph_handler->addFrontier(node, _rrg);
	}
	if (!_rrg.nodes[node].connected_to.empty()) {
		_global_graph_handler->continuePath(node, _rrg);
		_rrg.nodes[node].connected_to.clear();
	}
	for (int edge : _rrg.nodes[node].edges) {
		removeEdgeFromRemainingNode(edge, node);
	}
	_collision_checker->removeRetriableEdgesForNode(node);
	_rrg.nodes[node].edges.clear();
	_rrg.nodes[node].edge_counter = 0;
	_rrg.nodes[node].path_to_robot.clear();
	_rrg.nodes[node].gain = 0;
	_rrg.nodes[node].position.x = 0;
	_rrg.nodes[node].position.y = 0;
}

void GraphConstructor::removeEdgeFromRemainingNode(int edge, int node) {
	// ROS_INFO_STREAM("removeEdgeFromRemainingNode");
	int remaining_node =
			_rrg.edges[edge].first_node == _rrg.nodes[node].index ?
					_rrg.edges[edge].second_node : _rrg.edges[edge].first_node;
	_rrg.nodes[remaining_node].edges.erase(
			std::remove(_rrg.nodes[remaining_node].edges.begin(),
					_rrg.nodes[remaining_node].edges.end(), edge),
			_rrg.nodes[remaining_node].edges.end()); // remove edge from remaining node
}

void GraphConstructor::findAllConnectedAndDisconnectedNodes(
		std::vector<int> &disconnected_nodes, std::set<int> &connected_nodes) {
	for (int i = 0; i < disconnected_nodes.size(); i++) {
		ROS_INFO_STREAM("findAllConnectedAndDisconnectedNodes");
		// find disconnected nodes recursively
		int node = disconnected_nodes.at(i);
		//		ROS_INFO_STREAM("Disconnected Node " << node);
		for (int edge : _rrg.nodes[node].edges) {
			int neighbor_node =
					_rrg.edges[edge].first_node == node ?
							_rrg.edges[edge].second_node :
							_rrg.edges[edge].first_node;
			//			ROS_INFO_STREAM(
			//					"Has edge " << edge << " to node " << neighbor_node);
			if (isNextInPathToRobot(neighbor_node, node)) {
				_rrg.nodes[neighbor_node].cost_function = std::numeric_limits<
						double>::infinity();
				//				ROS_INFO_STREAM("Has also disconnected Node " << neighbor_node);
				if (std::find(disconnected_nodes.begin(),
						disconnected_nodes.end(), neighbor_node)
						== disconnected_nodes.end()) { // only add neighbor if not already present
					disconnected_nodes.push_back(neighbor_node);
				}
				connected_nodes.erase(neighbor_node);
			} else if (!_rrg.edges[edge].inactive
					&& !std::isinf(_rrg.nodes[neighbor_node].cost_function)) {
				//				ROS_INFO_STREAM(
				//						"Has still connected neighbor " << neighbor_node);
				connected_nodes.insert(neighbor_node);
			}
		}
	}
}

void GraphConstructor::checkCurrentGoal() {
	if (_current_goal_node == -1 && !_node_comparator->isEmpty()) {
		_current_goal_node = _node_comparator->getBestNode();
		_moved_to_current_goal = false;
		_local_goal_obsolete = false;
		_goal_obsolete = false;
		_updating_local_goal = false;
		_local_goal_updated = true;
		_pursued_local_goal = true;
		determineNextNodeInPath();
		ROS_INFO_STREAM("Current goal node set to " << _current_goal_node);
	}
}

void GraphConstructor::determineNextNodeInPath() {
	ROS_INFO_STREAM("determineNextNodeInPath");
	if (_current_goal_node >= 0
			&& _rrg.nodes[_current_goal_node].path_to_robot.size() > 1) {
		_next_node = _rrg.nodes[_current_goal_node].path_to_robot.at(1); // next goal on the path to the robot
		_edge_to_next_node = _graph_path_calculator->findExistingEdge(_rrg,
				_rrg.nearest_node, _next_node);
		if (_edge_to_next_node >= 0)
			return;
	}
	_next_node = -1;
}

void GraphConstructor::resetNextNodeInPath() {
	_next_node = -1;
	_edge_to_next_node = -1;
}

bool GraphConstructor::determineNearestNodeToRobot(geometry_msgs::Point pos) {
	if (pos.x != _last_robot_pos.x || pos.y != _last_robot_pos.y
			|| pos.z != _last_robot_pos.z) {
		_last_robot_pos = pos;
		double min_distance;
		int nearest_node;
		_graph_searcher->findNearestNeighbour(pos, min_distance, nearest_node);
		if (nearest_node != _rrg.nearest_node) { // if nearest node changed
			if (_next_node != -1 && nearest_node != _next_node) {
				// projection of robot pos on the edge between nearest and next node
				double projection_on_edge =
						((pos.x - _rrg.nodes[_rrg.nearest_node].position.x)
								* (_rrg.nodes[_next_node].position.x
										- _rrg.nodes[_rrg.nearest_node].position.x)
								+ (pos.y
										- _rrg.nodes[_rrg.nearest_node].position.y)
										* (_rrg.nodes[_next_node].position.y
												- _rrg.nodes[_rrg.nearest_node].position.y))
								/ pow(_rrg.edges[_edge_to_next_node].length, 2);
				if (projection_on_edge > 0 && projection_on_edge < 1) { // if projection is outside these bounds, the robot is outside the line segment
					double distance_to_edge =
							pow(
									(pos.x
											- (_rrg.nodes[_rrg.nearest_node].position.x
													+ projection_on_edge
															* (_rrg.nodes[_next_node].position.x
																	- _rrg.nodes[_rrg.nearest_node].position.x))),
									2)
									+ pow(
											(pos.y
													- (_rrg.nodes[_rrg.nearest_node].position.y
															+ projection_on_edge
																	* (_rrg.nodes[_next_node].position.y
																			- _rrg.nodes[_rrg.nearest_node].position.y))),
											2);
					if (distance_to_edge <= min_distance) { // robot is closer to edge than to nearest node, keep current one
						return false;
					}
				}
			}
			_moved_to_current_goal = true;
			_node_comparator->robotMoved();
			_rrg.nearest_node = nearest_node;
			_distance_to_nearest_node_squared = min_distance;
			addNodeToLastThreeNodesPath(nearest_node);
			return true;
		} else { // nearest node remained the same, just update distance
			double recalculate_distance = _robot_radius_squared / SQRT10;
			if (nearest_node == _current_goal_node
					&& _distance_to_nearest_node_squared > recalculate_distance
					&& min_distance <= recalculate_distance) { // update current goal node gain when 1/10 robot radius away
				_nodes_to_reupdate.clear();
				_nodes_to_reupdate.push_back(_node_comparator->getBestNode());
			}
			_distance_to_nearest_node_squared = min_distance;
		}
	}
	return false;
}

void GraphConstructor::publishExplorationGoalObsolete() {
	rrg_nbv_exploration_msgs::ExplorationGoalObsolete msg;
	if ((!_pursuing_global_goal
			&& ((!_node_comparator->isEmpty()
					&& _node_comparator->getBestNode() != _current_goal_node)
					|| _local_goal_obsolete))
			|| (_pursuing_global_goal
					&& (_global_frontier_obsolete
							|| (!_reached_frontier_goal
									&& _current_goal_node != -1)))) {
		//		if (!_goal_obsolete) {
		//			_goal_obsolete = true;
		//		ROS_WARN_STREAM(
		//				(!_pursuing_global_goal ? "Local " : "Global ")<< "goal obsolete" ", best goal: " << (_node_comparator->isEmpty() ? -1 : _node_comparator->getBestNode()) << " current goal: " << _current_goal_node << " explored current goal node by update: " << _local_goal_obsolete << " reached frontier goal: " << _reached_frontier_goal << " global frontier obsolete: " << _global_frontier_obsolete);
		//		}
		msg.goal_obsolete = true;
	} else {
		msg.goal_obsolete = false;
	}
	msg.goal_updated =
			_pursuing_global_goal ? _global_goal_updated : _local_goal_updated;
	_exploration_goal_obsolete_publisher.publish(msg);
}

void GraphConstructor::updateNodes(geometry_msgs::Point center_node) {
	std::vector<std::pair<int, double>> updatable_nodes =
			_graph_searcher->searchInRadius(center_node, _radius_search_range);
	for (auto iterator : updatable_nodes) {
		if (_rrg.nodes[iterator.first].status
				!= rrg_nbv_exploration_msgs::Node::EXPLORED
				&& _rrg.nodes[iterator.first].status
						!= rrg_nbv_exploration_msgs::Node::FAILED
				&& _rrg.nodes[iterator.first].status
						!= rrg_nbv_exploration_msgs::Node::INACTIVE
				&& !std::isinf(_rrg.nodes[iterator.first].cost_function)) {
			_nodes_to_update.push_back(iterator.first);
		}
		if (_rrg.nodes[iterator.first].status
				!= rrg_nbv_exploration_msgs::Node::FAILED
				&& _rrg.nodes[iterator.first].status
						!= rrg_nbv_exploration_msgs::Node::INACTIVE
				&& _rrg.nodes[iterator.first].retry_inflation) { // retry previously failed inflation
			std::vector<int> engulfed_nodes =
					_collision_checker->inflateExistingNode(_rrg,
							iterator.first, _robot_pose, _nodes_to_update,
							_sort_nodes_to_update);
			if (engulfed_nodes.size() > 0)
				pruneEngulfedNodes(engulfed_nodes);
			_graph_searcher->rebuildIndex(_rrg);
			if (!_rrg.nodes.at(iterator.first).retry_inflation) // only prune if inflation is finished for this node
				_global_graph_handler->pruneFrontiersAndPathsAroundNewNode(_rrg,
						iterator.first);
		}
	}
}

void GraphConstructor::pruneEngulfedNodes(std::vector<int> engulfed_nodes) {
	ROS_INFO_STREAM("Prune " << engulfed_nodes.size() << " engulfed nodes");
	std::set<int> pruned_nodes;
	std::set<int> pruned_edges;
	for (auto engulfed_node : engulfed_nodes) {
		pruned_nodes.insert(engulfed_node);
		_rrg.nodes.at(engulfed_node).status =
				rrg_nbv_exploration_msgs::Node::INACTIVE;
		ROS_INFO_STREAM("Node to be pruned: " << engulfed_node);
		for (int edge : _rrg.nodes.at(engulfed_node).edges) {
			ROS_INFO_STREAM("Edge " << edge << " to be pruned");
			_rrg.edges[edge].inactive = true;
			pruned_edges.insert(edge);
		}
	}
	handlePrunedNodes(pruned_nodes);
	handlePrunedEdges(pruned_edges);
}

void GraphConstructor::sortNodesToUpdateByDistanceToRobot() {
	_nodes_to_update.sort([this](int node_one, int node_two) {
		return compareNodeDistancesToRobot(node_one, node_two);
	});
	_sort_nodes_to_update = false;
}

bool GraphConstructor::compareNodeDistancesToRobot(const int &node_one,
		const int &node_two) {
	return _rrg.nodes.at(node_one).distance_to_robot
			<= _rrg.nodes.at(node_two).distance_to_robot;
}

void GraphConstructor::publishNodeToUpdate() {
	if (!_nodes_to_update.empty()) {
		if (_sort_nodes_to_update)
			sortNodesToUpdateByDistanceToRobot();
		rrg_nbv_exploration_msgs::NodeToUpdate msg;
		msg.node = _rrg.nodes[_nodes_to_update.front()];
		msg.force_update = _nodes_to_update.front() == _last_goal_node;
		_node_to_update_publisher.publish(msg);
	} else if (_reupdate_nodes && !_nodes_to_reupdate.empty()) { // if list is empty, re-update nodes ordered ascending by their distance to the robot
		rrg_nbv_exploration_msgs::NodeToUpdate msg;
		int node = _nodes_to_reupdate.front();
		_nodes_to_reupdate.erase(_nodes_to_reupdate.begin());
		if (_rrg.nodes[node].distance_to_robot > _sensor_range) { // don't re-update nodes outside of sensor range
			_nodes_to_reupdate.clear();
			return;
		}
		msg.node = _rrg.nodes.at(node);
		msg.force_update = false;
		_node_to_update_publisher.publish(msg);
	}
}

void GraphConstructor::handleCurrentLocalGoalFinished() {
	geometry_msgs::Point update_center;
	switch (_rrg.nodes[_current_goal_node].status) {
	case rrg_nbv_exploration_msgs::Node::EXPLORED:
		ROS_INFO_STREAM("RNE goal " << _current_goal_node << " explored");
		_node_comparator->removeNode(_current_goal_node);
		update_center = _rrg.nodes[_current_goal_node].position;
		updateNodes(update_center);
		resetNextNodeInPath();
		tryFailedNodesRecovery();
		_consecutive_failed_goals = 0;
		break;
	case rrg_nbv_exploration_msgs::Node::VISITED:
		ROS_INFO_STREAM("RNE goal " << _current_goal_node << " visited");
		_node_comparator->removeNode(_current_goal_node);
		_sort_nodes_to_update = true;
		_rrg.nodes[_current_goal_node].gain = 0;
		_rrg.nodes[_current_goal_node].heading_change_to_robot_best_view =
				_rrg.nodes[_current_goal_node].heading_change_to_robot;
		_rrg.nodes[_current_goal_node].reward_function = 0;
		update_center = _rrg.nodes[_current_goal_node].position;
		updateNodes(update_center);
		resetNextNodeInPath();
		tryFailedNodesRecovery();
		_consecutive_failed_goals = 0;
		break;
	case rrg_nbv_exploration_msgs::Node::ABORTED:
		ROS_INFO_STREAM("RNE goal " << _current_goal_node << " aborted");
		_rrg.nodes[_current_goal_node].status =
				_rrg.nodes[_current_goal_node].gain > 0 ?
						rrg_nbv_exploration_msgs::Node::INITIAL :
						rrg_nbv_exploration_msgs::Node::EXPLORED;
		if (_moved_to_current_goal) { // do not update nodes because of constant re-updating
			if (!_reupdate_nodes) {
				ROS_INFO("update nodes");
				_node_comparator->removeNode(_current_goal_node);
				_sort_nodes_to_update = true;
				update_center = _last_robot_pos;
				updateNodes(update_center);
			}
			tryFailedNodesRecovery();
		}
		resetNextNodeInPath();
		break;
	case rrg_nbv_exploration_msgs::Node::FAILED:
		ROS_INFO_STREAM("RNE goal " << _current_goal_node << " failed");
		_node_comparator->removeNode(_current_goal_node);
		_failed_nodes_to_recover.push_back(_current_goal_node);
		_rrg.nodes[_current_goal_node].gain = 0;
		_rrg.nodes[_current_goal_node].heading_change_to_robot_best_view =
				_rrg.nodes[_current_goal_node].heading_change_to_robot;
		_rrg.nodes[_current_goal_node].reward_function = 0;
		if (++_consecutive_failed_goals >= _max_consecutive_failed_goals) {
			ROS_WARN_STREAM("Exploration aborted, robot stuck");
			_rrg.node_counter = -1; // for evaluation purposes
			stopExploration();
		}
		if (_moved_to_current_goal) {
			update_center = _last_robot_pos;
			updateNodes(update_center);
		}
		resetNextNodeInPath();
		break;
	case rrg_nbv_exploration_msgs::Node::INACTIVE:
		ROS_ERROR_STREAM("RNE goal " << _current_goal_node << " is inactive!");
		break;
	default:
		// active or waiting (should not occur)
		ROS_INFO_STREAM(
				"RNE goal " << _current_goal_node << " active or waiting");
		return;
	}
	_last_goal_node = _current_goal_node;
	_current_goal_node = -1;
}

void GraphConstructor::updatedNodeCallback(
		const rrg_nbv_exploration_msgs::Node::ConstPtr &updated_node) {
	ROS_INFO_STREAM("+++++ updatedNodeCallback " << updated_node->index);
	if (updated_node->index >= _rrg.node_counter
			|| _rrg.nodes.at(updated_node->index).status
					== rrg_nbv_exploration_msgs::Node::INACTIVE) {
		ROS_WARN_STREAM(
				"Node " << updated_node->index << " was updated but already removed from the local graph");
		_nodes_to_update.remove(updated_node->index);
		_last_updated_node = -1;
		return;
	}
	if (updated_node->index != _last_updated_node
			|| _rrg.nodes[_last_updated_node].gain != updated_node->gain
			|| _rrg.nodes[_last_updated_node].best_yaw != updated_node->best_yaw
			|| _rrg.nodes[_last_updated_node].status != updated_node->status) {
		bool check_waypoints = _rrg.nodes[updated_node->index].gain == -1;
		_rrg.nodes[updated_node->index].gain = updated_node->gain;
		_rrg.nodes[updated_node->index].best_yaw = updated_node->best_yaw;
		_rrg.nodes[updated_node->index].status = updated_node->status;
		_rrg.nodes[updated_node->index].position.z = updated_node->position.z;
		_last_updated_node = updated_node->index;
		_nodes_to_update.remove(updated_node->index);
		_node_comparator->removeNode(updated_node->index);
		if (updated_node->status != rrg_nbv_exploration_msgs::Node::EXPLORED
				&& updated_node->status
						!= rrg_nbv_exploration_msgs::Node::FAILED
				&& updated_node->status
						!= rrg_nbv_exploration_msgs::Node::INACTIVE) {
			_rrg.nodes[updated_node->index].heading_change_to_robot_best_view =
					_graph_path_calculator->calculateHeadingChangeToBestView(
							_rrg.nodes[updated_node->index]);
			_rrg.nodes[updated_node->index].cost_function =
					_graph_path_calculator->calculateCostFunction(
							_rrg.nodes[updated_node->index]);
			_node_comparator->addNode(updated_node->index);
			_sort_nodes_to_update = true;
			_local_exploration_finished_timer.stop();
		} else {
			_rrg.nodes[updated_node->index].gain = 0;
			_rrg.nodes[updated_node->index].heading_change_to_robot_best_view =
					_rrg.nodes[updated_node->index].heading_change_to_robot;
			_rrg.nodes[updated_node->index].reward_function = 0.0;
			if (updated_node->status == rrg_nbv_exploration_msgs::Node::EXPLORED
					&& _current_goal_node == updated_node->index
					&& _node_comparator->isEmpty()) { // current goal is explored, change goal
				_local_goal_obsolete = true;
			}
		}
		if (check_waypoints) { // node was just added
			std::vector<geometry_msgs::Point> frontier_viewpoints =
					_global_graph_handler->pruneFrontiersAndPathsAroundNewNode(
							_rrg, updated_node->index); // if neighbor of new node is connected to frontiers, try to rewire paths
			_new_node_positions.insert(_new_node_positions.end(),
					frontier_viewpoints.begin(), frontier_viewpoints.end());
		}
		publishNodeToUpdate(); // if gain calculation is faster than update frequency, this needs to be called
	}
	ROS_INFO_STREAM("----- updatedNodeCallback " << updated_node->index);
}

void GraphConstructor::tryFailedNodesRecovery() {
	_failed_nodes_to_recover.erase(
			std::remove_if(_failed_nodes_to_recover.begin(),
					_failed_nodes_to_recover.end(),
					[this](int node) {
						int collision =
								_collision_checker->collisionCheckForFailedNode(
										_rrg, node);
						if (collision
								== CollisionChecker::Collisions::unknown) {
							return false; // keep node and try to recover again
						} else if (collision
								== CollisionChecker::Collisions::empty) {
							_rrg.nodes[node].status =
									rrg_nbv_exploration_msgs::Node::INITIAL;
							_collision_checker->findBestConnectionForNode(_rrg,
									_rrg.nodes[node], _robot_pose, false);
							_graph_path_calculator->updatePathsToRobot(node,
									_rrg, _robot_pose, false, _nodes_to_update,
									_sort_nodes_to_update); // check if recovered connection could improve other distances
							if (!std::isinf(_rrg.nodes[node].cost_function)) { // check if node is reachable
								_nodes_to_update.push_back(node); // recalculate node gain
								_sort_nodes_to_update = true;
							}
						}
						return true;
					}),
			_failed_nodes_to_recover.end());
	_collision_checker->retryEdges(_rrg, _robot_pose, _nodes_to_update,
			_sort_nodes_to_update); // try to rebuild failed edges
}

void GraphConstructor::addNodeToLastThreeNodesPath(int node) {
	if (!_reupdate_nodes)
		return;
	auto it = std::find(_last_three_nodes_path.begin(),
			_last_three_nodes_path.end(), node);
	if (it == _last_three_nodes_path.end()) {
		if (_last_three_nodes_path.size() >= 3) {
			_last_three_nodes_path.erase(_last_three_nodes_path.begin());
		}
		_last_three_nodes_path.push_back(node);
		_nodes_to_reupdate.clear();
		_nodes_to_reupdate = _node_comparator->getListOfNodes();
		std::sort(_nodes_to_reupdate.begin(), _nodes_to_reupdate.end(),
				[this](int node_one, int node_two) {
					return compareNodeDistancesToRobot(node_one, node_two);
				});
	} else {
		std::iter_swap(it, _last_three_nodes_path.rbegin());
	}
}

void GraphConstructor::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr &map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
	updateMapDimensions();
}

void GraphConstructor::updateMapDimensions() {
	_octree->getMetricMin(_map_min_bounding[0], _map_min_bounding[1],
			_map_min_bounding[2]);
	_octree->getMetricMax(_map_max_bounding[0], _map_max_bounding[1],
			_map_max_bounding[2]);
}

void GraphConstructor::switchFromLocalToGlobalExploration() {
	if (!_pursued_local_goal) { // if a current frontier goal is active, it became obsolete
		_global_frontier_obsolete = true;
		publishExplorationGoalObsolete();
		_global_frontier_obsolete = false;
		_pursued_local_goal = true;
	}
	_local_running = false;
}

void GraphConstructor::localExplorationFinishedTimerCallback(
		const ros::TimerEvent &event) {
	_local_exploration_finished_timer.stop();
	ROS_INFO_STREAM(
			"Local exploration without goals, switch to global exploration");
	switchFromLocalToGlobalExploration();
}

bool GraphConstructor::requestGoal(
		rrg_nbv_exploration_msgs::RequestGoal::Request &req,
		rrg_nbv_exploration_msgs::RequestGoal::Response &res) {
//	ROS_INFO_STREAM(
//			"Request "<< (_local_running ? "local " : "global ")<<"goal, updated: " << _local_goal_updated);
	res.exploration_finished = !_running;
	if (_local_running) { // local goal
		res.goal_available = _local_goal_updated && _current_goal_node != -1;
		if (_current_goal_node != -1) {
			if (_rrg.nodes[_current_goal_node].status
					== rrg_nbv_exploration_msgs::Node::VISITED) {
				_rrg.nodes[_current_goal_node].status =
						rrg_nbv_exploration_msgs::Node::ACTIVE_VISITED;
			} else if (_rrg.nodes[_current_goal_node].status
					== rrg_nbv_exploration_msgs::Node::INITIAL) {
				_rrg.nodes[_current_goal_node].status =
						rrg_nbv_exploration_msgs::Node::ACTIVE;
			}
			res.goal = _rrg.nodes.at(_current_goal_node).position;
			res.best_yaw = _graph_path_calculator->determineGoalYaw(
					_current_goal_node, _rrg, _last_robot_pos, !_running);
			_local_goal_updated = false;
			_pursuing_global_goal = false;
		}
	} else { // global goal
		geometry_msgs::Point goal;
		double yaw;
		if (_global_graph_handler->getFrontierGoal(goal, yaw,
				_robot_pose.position)) {
			res.goal_available = true;
			res.goal = goal;
			res.best_yaw = yaw;
			_global_goal_updated = false;
			_pursuing_global_goal = true;
			_reached_frontier_goal = false;
		} else {
			res.goal_available = false;
		}
	}
	return true;
}

bool GraphConstructor::requestPath(
		rrg_nbv_exploration_msgs::RequestPath::Request &req,
		rrg_nbv_exploration_msgs::RequestPath::Response &res) {
	if (_local_running) { // local path
		if (_current_goal_node != -1) {
			std::vector<geometry_msgs::PoseStamped> rrg_path;
			_graph_path_calculator->getLocalNavigationPath(rrg_path, _rrg,
					_current_goal_node, _robot_pose.position);
			res.path = rrg_path;
			return true;
		}
	} else { // global path
		std::vector<geometry_msgs::PoseStamped> path;
		if (_global_graph_handler->getFrontierPath(path,
				_robot_pose.position)) {
			//			_graph_path_calculator->getLocalNavigationPath(path, _rrg,
			//					connecting_node, _robot_pose.position);
			res.path = path;
			return true;
		}
	}
	return false;
}

bool GraphConstructor::updateCurrentGoal(
		rrg_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
		rrg_nbv_exploration_msgs::UpdateCurrentGoal::Response &res) {
	if (!_pursuing_global_goal) { // local goal
		if (_current_goal_node != -1 && !_updating_local_goal) {
			_updating_local_goal = true;
			_rrg.nodes[_current_goal_node].status = req.status;
			handleCurrentLocalGoalFinished();
		}
	} else { // global goal
		if (!_updating_global_goal) {
			_updating_global_goal = true;
			if (req.status == rrg_nbv_exploration_msgs::Node::VISITED
					|| req.status == rrg_nbv_exploration_msgs::Node::ABORTED) { // reached frontier goal or aborted it for new one
				ROS_INFO_STREAM("Frontier goal reached");
				_reached_frontier_goal = true;
				_consecutive_failed_goals = 0;
			} else { // failed to reach frontier goal
				ROS_INFO_STREAM("Navigation to frontier goal failed");
				if (++_consecutive_failed_goals
						>= _max_consecutive_failed_goals) {
					ROS_WARN_STREAM("Exploration aborted, robot stuck");
					_rrg.node_counter = -1; // for evaluation purposes
					stopExploration();
				}
				if (_global_graph_handler->frontierFailed()) {
					ROS_INFO_STREAM("Exploration finished");
					_running = false;
				}
			}
		}
	}
	res.success = true;
	res.message = "Changed current goal's status";
	return true;
}

void GraphConstructor::startExploration() {
	initExploration(_graph_path_calculator->getRobotPose().position);
	_running = true;
	_local_running = true;
}

void GraphConstructor::stopExploration() {
	_running = false;
	_local_running = false;
}

bool GraphConstructor::setRrgState(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (req.data) {
		if (_running) {
			res.success = false;
			res.message = "Tree construction already running";
		} else {
			startExploration();
			res.success = true;
			res.message = "Tree construction started";
		}
	} else {
		if (!_running) {
			res.success = false;
			res.message = "Tree construction already stopped";
		} else {
			stopExploration();
			res.success = true;
			res.message = "Tree construction stopped";
		}
	}
	return true;
}

bool GraphConstructor::getRrgState(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	res.success = _running;
	res.message = "Tree construction status sent";
	return true;
}

bool GraphConstructor::resetRrgState(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	_running = false;
	initExploration(geometry_msgs::Point());
	res.message = true;
	res.message = "Tree reset";
	return true;
}

void GraphConstructor::dynamicReconfigureCallback(
		rrg_nbv_exploration::GraphConstructorConfig &config, uint32_t level) {
	_local_graph_radius = std::max(config.local_graph_radius,
			2 * _robot_radius); // cannot fall below the robot diameter
	_local_graph_radius_squared = pow(_local_graph_radius + _robot_radius, 2); // add buffer to pruning
	_local_sampling_radius = std::min(_local_graph_radius,
			config.local_sampling_radius); // cannot exceed local graph radius
	_node_comparator->dynamicReconfigureCallback(config, level);
	_graph_path_calculator->dynamicReconfigureCallback(config, level);
	_global_graph_handler->dynamicReconfigureCallback(config, level);
}

}
