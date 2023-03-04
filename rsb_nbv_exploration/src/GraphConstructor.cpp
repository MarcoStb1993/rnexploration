#include <rsb_nbv_exploration/GraphConstructor.h>

namespace rsb_nbv_exploration {
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
	private_nh.param("global_exploration_active", _global_exploration_active,
			true);
	private_nh.param("initialization_node_distance",
			_initialization_node_distance, 0.0);
	private_nh.param("measure_algorithm_runtime", _measure_algorithm_runtime,
			false);

	ros::NodeHandle nh("rne");
	_rrg_publisher = nh.advertise<rsb_nbv_exploration_msgs::Graph>("rrg", 1);
	_rne_state_publisher = nh.advertise<std_msgs::Bool>("state", 1);
	_node_to_update_publisher = nh.advertise<
			rsb_nbv_exploration_msgs::NodeToUpdate>("node_to_update", 1);
	_updated_node_subscriber = nh.subscribe("updated_node", 1,
			&GraphConstructor::updatedNodeCallback, this);
	_exploration_goal_obsolete_publisher = nh.advertise<
			rsb_nbv_exploration_msgs::ExplorationGoalObsolete>(
			"explorationGoalObsolete", 1);
	_request_goal_service = nh.advertiseService("requestGoal",
			&GraphConstructor::requestGoal, this);
	_request_path_service = nh.advertiseService("requestPath",
			&GraphConstructor::requestPath, this);
	_update_current_goal_service = nh.advertiseService("updateCurrentGoal",
			&GraphConstructor::updateCurrentGoal, this);
	if (_measure_algorithm_runtime) {
		_rne_runtime_publisher = nh.advertise<std_msgs::Duration>("rne_runtime",
				1);
	}

	_local_exploration_finished_timer = _nh.createTimer(
			ros::Duration(_local_exploration_finished_timer_duration),
			&GraphConstructor::localExplorationFinishedTimerCallback, this,
			false, false);

	_set_rne_state_service = nh.advertiseService("setRneState",
			&GraphConstructor::setRrgState, this);
	_get_rne_state_service = nh.advertiseService("getRneState",
			&GraphConstructor::getRrgState, this);
	_reset_rne_state_service = nh.advertiseService("resetRneState",
			&GraphConstructor::resetRrgState, this);

	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&GraphConstructor::convertOctomapMsgToOctree, this);

	if (_global_exploration_active) {
		_local_graph_radius = std::max(_local_graph_radius, 2 * _robot_radius);	// cannot fall below the robot diameter
		_local_graph_pruning_radius_squared = pow(
				_local_graph_radius + _robot_radius, 2);// add buffer to pruning
		_local_sampling_radius = std::min(_local_graph_radius,
				_local_sampling_radius); // cannot exceed local graph radius
		_global_graph_handler.reset(new GlobalGraphHandler());
		_local_sampling_radius = std::min(_local_sampling_radius,
				_local_graph_radius); // cannot sample outside local area
	}

	_graph_searcher.reset(new GraphSearcher());
	_collision_checker.reset(new CollisionChecker());
	_graph_path_calculator.reset(new GraphPathCalculator());
	_node_comparator.reset(new NodeComparator());
	_running = false;
	_local_running = false;
}

void GraphConstructor::initExploration(const geometry_msgs::Pose &seed) {
	_pursuing_global_goal = false;
	_reached_frontier_goal = false;
	_global_goal_updated = true;
	_updating_global_goal = false;
	initLocalGraph(seed.position);
	resetHelperClasses();
	if (_initialization_node_distance > 0) {
		ROS_INFO_STREAM("Add initialization node");
		rsb_nbv_exploration_msgs::Node init_node;
		init_node.index = 1;
		init_node.position = seed.position;
		init_node.position.z = _sensor_height;
		//add node in front of the robot's heading
		tf2::Quaternion q(seed.orientation.x, seed.orientation.y,
				seed.orientation.z, seed.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		init_node.position.x += cos(yaw) * _initialization_node_distance;
		init_node.position.y += sin(yaw) * _initialization_node_distance;
		init_node.status = rsb_nbv_exploration_msgs::Node::INITIAL;
		init_node.gain = -1;
		init_node.reward_function = 0;
		init_node.edges.push_back(0);
		init_node.edge_counter++;
		init_node.radius = _robot_radius;
		init_node.retry_inflation = true;
		init_node.path_to_robot.push_back(1);
		init_node.path_to_robot.push_back(0);
		_rrg.nodes.push_back(init_node);
		_rrg.node_counter++;
		// add edge between init and root node
		rsb_nbv_exploration_msgs::Edge edge;
		edge.index = 0;
		edge.first_node = 0;
		edge.second_node = 1;
		edge.inactive = false;
		edge.length = 2.0;
		edge.yaw = (int) (yaw * 180 / M_PI);
		_rrg.edges.push_back(edge);
		_rrg.edge_counter++;
		// update root node's edges
		_rrg.nodes.front().edges.push_back(0);
		_rrg.nodes.front().edge_counter++;
		_nodes_to_update.push_back(1);
		_initialization_node_distance = 0.0; //only add for first local graph
	}
	if (_global_exploration_active) {
		_global_graph_handler->initialize(_rrg.nodes.at(0),
				_graph_path_calculator, _graph_searcher, _collision_checker);
	}
	_generator.seed(time(NULL));
	if (_measure_algorithm_runtime) {
		_algorithm_runtime = ros::Duration(0, 0);
	}
}

void GraphConstructor::initLocalGraph(
		const geometry_msgs::Point &root_position) {
	_nodes_to_update.clear();
	_nodes_to_reupdate.clear();
	_last_three_nodes_path.clear();
	_new_node_positions.clear();
	_rrg.header.frame_id = "map";
	_rrg.ns = "rrg";
	_rrg.nodes.clear();
	_rrg.edges.clear();
	_rrg.node_counter = 0;
	_rrg.edge_counter = 0;
	rsb_nbv_exploration_msgs::Node root;
	root.position = root_position;
	root.position.z += _sensor_height;
	root.status = rsb_nbv_exploration_msgs::Node::VISITED;
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
	_pursuing_local_goal = false;
	_local_goal_obsolete = false;
	_current_goal_node = -1;
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
		if (_local_running) {
			bool new_nearest_node = determineNearestNodeToRobot(
					_robot_pose.position); // check if nearest node to robot changed which means robot moved
			expandGraph(!new_nearest_node);
			if (new_nearest_node) {	// robot moved, update paths
				_graph_path_calculator->updatePathsToRobot(_rrg.nearest_node,
						_rrg, _robot_pose, true, _nodes_to_update,
						_sort_nodes_to_update);
				if (_global_exploration_active) {
					pruneLocalGraph(); // remove nodes from local RRG that are too far away after updating paths
				}
				determineNextNodeInPath();
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
		} else if (_global_exploration_active && !_local_running) {
			if (!_global_graph_handler->checkIfNextFrontierWithPathIsValid()) {
				if (!_global_graph_handler->calculateNextFrontierGoal(_rrg)) { // exploration finished
					ROS_INFO_STREAM("Exploration finished");
					_running = false;
				} else {
					_global_goal_updated = true;
					_updating_global_goal = false;
				}
			} else if (_reached_frontier_goal
					|| _global_graph_handler->updateClosestWaypoint(
							_robot_pose.position)) { // frontier reached
				geometry_msgs::Point frontier;
				std::vector<int> connected_paths =
						_global_graph_handler->targetReached(frontier);
				if (connected_paths.empty()) { // reached frontier must be origin, auto homing is active->exploration finished
					ROS_INFO_STREAM("Exploration finished");
					_running = false;
				} else {
					switchFromGlobalToToLocalExploration(frontier,
							connected_paths);
				}
			}
		}
		publishExplorationGoalObsolete();
	}
	if (_measure_algorithm_runtime) {
		if (_running) {
			_algorithm_runtime += ros::Time::now() - _rrg.header.stamp;
		} else {
			_algorithm_runtime = ros::Duration(0, 0);
		}
		std_msgs::Duration runtime;
		runtime.data = _algorithm_runtime;
		_rne_runtime_publisher.publish(runtime);
	}
	_rrg_publisher.publish(_rrg);
	if (_global_exploration_active) {
		_global_graph_handler->publishGlobalGraph();
	}
	std_msgs::Bool rne_state;
	rne_state.data = _running;
	_rne_state_publisher.publish(rne_state);
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
		insertNewNode(true, new_node_position, update_paths, true);
	}
	_new_node_positions.clear();
}

void GraphConstructor::insertNewNode(bool sampling_success,
		const geometry_msgs::Point &rand_sample, bool update_paths,
		bool unmovable_point) {
	rsb_nbv_exploration_msgs::Node node;
	if (sampling_success
			&& _collision_checker->steer(_rrg, node, rand_sample, _robot_pose,
					unmovable_point)) {
		if (std::isinf(node.cost_function) == 0) { // do not update unreachable nodes
			if (update_paths)
				_graph_path_calculator->updatePathsToRobot(node.index, _rrg,
						_robot_pose, false, _nodes_to_update,
						_sort_nodes_to_update); // check if new connection could improve other distances
			_nodes_to_update.push_back(node.index);
			_sort_nodes_to_update = true;
		}
		_graph_searcher->rebuildIndex(_rrg);
//		_local_exploration_finished_timer.stop();
	}
}

bool GraphConstructor::samplePoint(geometry_msgs::Point &rand_sample) {
	if (_global_exploration_active) {
		return samplePointInRadius(rand_sample, _robot_pose.position,
				std::min(_sensor_range, _local_graph_radius));
	} else {
		std::uniform_real_distribution<double> x_distribution(
				_map_min_bounding[0], _map_max_bounding[0]);
		std::uniform_real_distribution<double> y_distribution(
				_map_min_bounding[1], _map_max_bounding[1]);
		rand_sample.x = x_distribution(_generator);
		rand_sample.y = y_distribution(_generator);
		return true;
	}
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
	std::set<int> pruned_nodes;
	std::set<int> pruned_edges;
	std::vector<int> disconnected_nodes = findOutOfLocalGraphRadiusNodes(
			pruned_nodes, pruned_edges);
	std::set<int> connected_nodes; // nodes with a path to the robot and an edge to a disconnected node
	findAllConnectedAndDisconnectedNodes(disconnected_nodes, connected_nodes);
	for (auto node : connected_nodes) { // start path update from each connected nodes
		_graph_path_calculator->updatePathsToRobot(node, _rrg, _robot_pose,
				false, _nodes_to_update, _sort_nodes_to_update);
	}
	for (auto node : disconnected_nodes) {
		if (std::isinf(_rrg.nodes[node].cost_function) != 0) { // check if disconnected nodes are still disconnected and deactivate them
			if (_rrg.nodes.at(node).status
					== rsb_nbv_exploration_msgs::Node::FAILED
					&& !_rrg.nodes.at(node).connected_to.empty()
					&& _collision_checker->collisionCheckForFailedNode(_rrg,
							node, false)
							== CollisionChecker::Collisions::empty) { //try to recover node with connected paths before pruning
				_rrg.nodes[node].status =
						rsb_nbv_exploration_msgs::Node::INITIAL;
				_collision_checker->findBestConnectionForNode(_rrg,
						_rrg.nodes[node], _robot_pose, false);
			}
			pruned_nodes.insert(node);
			_rrg.nodes[node].status = rsb_nbv_exploration_msgs::Node::INACTIVE;
			for (int edge : _rrg.nodes[node].edges) {
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
}

std::vector<int> GraphConstructor::findOutOfLocalGraphRadiusNodes(
		std::set<int> &pruned_nodes, std::set<int> &pruned_edges) {
	std::set<int> disconnected_nodes_set;
	for (auto &node : _rrg.nodes) {
		if (node.status != rsb_nbv_exploration_msgs::Node::INACTIVE) {
			double distance_squared = pow(
					_robot_pose.position.x - node.position.x, 2)
					+ pow(_robot_pose.position.y - node.position.y, 2);
			if (distance_squared > _local_graph_pruning_radius_squared) { // remove node from local graph
				pruned_nodes.insert(node.index);
				_rrg.nodes[node.index].status =
						rsb_nbv_exploration_msgs::Node::INACTIVE;
				disconnected_nodes_set.erase(node.index); // remove in case it was already added
				for (int edge : node.edges) {
					_rrg.edges[edge].inactive = true;
					pruned_edges.insert(edge);
					int remaining_node =
							_rrg.edges[edge].first_node == node.index ?
									_rrg.edges[edge].second_node :
									_rrg.edges[edge].first_node;
					if (isNextInPathToRobot(remaining_node, node.index)) {
						_rrg.nodes[remaining_node].cost_function =
								std::numeric_limits<double>::infinity();
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
	bool removed_edges = false;
	auto pruned_edge = pruned_edges.rbegin();
	while (pruned_edge != pruned_edges.rend()) { // remove or add inactive edges
		if (*pruned_edge == _rrg.edge_counter - 1) {
			int next_pruned_edge =
					std::next(pruned_edge) == pruned_edges.rend() ?
							-1 : *std::next(pruned_edge); // if last pruned edge, try to remove inactive edges up to index 0
			for (int i = *pruned_edge; i > next_pruned_edge; i--) {
				if (_rrg.edges.at(i).inactive) {
					_rrg.edges.pop_back();
					_rrg.edge_counter--;
					removed_edges = true;
				} else {
					break;
				}
			}
		} else {
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

bool GraphConstructor::hasRelevantNodeWithoutPath(
		const std::vector<int> &nodes) {
	for (auto n : nodes) {
		if ((_rrg.nodes.at(n).connected_to.size() > 0
				|| _rrg.nodes.at(n).gain > 0)
				&& _rrg.nodes.at(n).path_to_robot.empty()) {
			return true;
		}
	}
	return false;
}

void GraphConstructor::handlePrunedNodes(const std::set<int> &pruned_nodes) {
	std::vector<int> nodes(pruned_nodes.begin(), pruned_nodes.end());
	tryFailedNodesRecovery();
	if (hasRelevantNodeWithoutPath(nodes)) { //connect nodes to be pruned to the nearest node for adding frontiers and continuing paths
		for (auto n : nodes) {
			_graph_path_calculator->findPathToNearestNodeThroughFailedNodes(
					_rrg, n);
		}
	}

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
						== rsb_nbv_exploration_msgs::Node::INACTIVE) {
					_rrg.nodes.pop_back();
					_rrg.node_counter--;
					removed_nodes = true;
				} else {
					break;
				}
			}
		} else {
			_collision_checker->addAvailableNode(*pruned_node);
		}
		pruned_node++;
	}
	if (removed_nodes) {
		_collision_checker->removeDeletedAvailableNodes(_rrg.node_counter);
	}
}

bool GraphConstructor::sortByPathLength(int node_one, int node_two) {
	if (_rrg.nodes[node_one].path_to_robot.size()
			< _rrg.nodes[node_two].path_to_robot.size()) {
		return true;
	} else if (_rrg.nodes[node_one].path_to_robot.size()
			> _rrg.nodes[node_two].path_to_robot.size()) {
		return false;
	} else {
		if (_rrg.nodes[node_one].distance_to_robot
				< _rrg.nodes[node_two].distance_to_robot) {
			return true;
		} else if (_rrg.nodes[node_one].distance_to_robot
				> _rrg.nodes[node_two].distance_to_robot) {
			return false;
		} else {
			return node_one < node_two;
		}
	}
}

void GraphConstructor::findConnectedNodesWithGain(std::vector<int> &nodes) {
	std::set<int> possible_frontiers;
	for (auto node : nodes) {
		if (_rrg.nodes.at(node).gain > 0.0) {
			possible_frontiers.insert(node);
		}
	}
	while (!possible_frontiers.empty()) {
		std::vector<int> frontier;
		frontier.push_back(*possible_frontiers.begin());
		possible_frontiers.erase(possible_frontiers.begin());
		for (int i = 0; i < frontier.size(); i++) { // build frontier from possible frontier nodes
			for (auto edge : _rrg.nodes.at(frontier.at(i)).edges) {
				int neighbor_node =
						frontier.at(i) == _rrg.edges.at(edge).first_node ?
								_rrg.edges.at(edge).second_node :
								_rrg.edges.at(edge).first_node;
				if (_rrg.nodes.at(neighbor_node).gain > 0
						&& possible_frontiers.erase(neighbor_node) > 0) { // neighbor will also become a frontier
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
	}
}

void GraphConstructor::deactivateNode(int node) {
	if (node == _current_goal_node) {
		ROS_INFO_STREAM("Deactivated current goal node");
		_local_goal_obsolete = true;
	}
	if (node == _last_updated_node) {
		_last_updated_node = -1;
	}
	if (node == _last_goal_node) {
		_last_goal_node = -1;
	}
	removeNodeFromUpdateLists(node);
	if (_global_exploration_active) {
		if (_rrg.nodes[node].gain > 0) {
			_global_graph_handler->addFrontier(node, _rrg);
		}
		if (!_rrg.nodes[node].connected_to.empty()) {
			_global_graph_handler->continuePath(node, _rrg);
			_rrg.nodes[node].connected_to.clear();
		}
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
		// find disconnected nodes recursively
		int node = disconnected_nodes.at(i);
		for (int edge : _rrg.nodes[node].edges) {
			int neighbor_node =
					_rrg.edges[edge].first_node == node ?
							_rrg.edges[edge].second_node :
							_rrg.edges[edge].first_node;
			if (isNextInPathToRobot(neighbor_node, node)) {
				_rrg.nodes[neighbor_node].cost_function = std::numeric_limits<
						double>::infinity();
				if (std::find(disconnected_nodes.begin(),
						disconnected_nodes.end(), neighbor_node)
						== disconnected_nodes.end()) { // only add neighbor if not already present
					disconnected_nodes.push_back(neighbor_node);
				}
				connected_nodes.erase(neighbor_node);
			} else if (!_rrg.edges[edge].inactive
					&& std::isinf(_rrg.nodes[neighbor_node].cost_function)
							== 0) {
				connected_nodes.insert(neighbor_node);
			}
		}
	}
}

bool GraphConstructor::isNewGoalConnected() {
	if (std::isinf(_rrg.nodes.at(_current_goal_node).cost_function) != 0) {
		_current_goal_node = -1;
		_node_comparator->setSortList();
		_node_comparator->maintainList(_rrg);
		if (!_node_comparator->isEmpty()) {
			_current_goal_node = _node_comparator->getBestNode();
			if (std::isinf(_rrg.nodes.at(_current_goal_node).cost_function)
					!= 0) {
				_current_goal_node = -1;
				return false;
			}
		} else {
			return false;
		}

	}
	return true;
}

void GraphConstructor::checkCurrentGoal() {
	if (_current_goal_node == -1 && !_node_comparator->isEmpty()) {
		_current_goal_node = _node_comparator->getBestNode();
		if (!isNewGoalConnected()) {
			return;	//no new goal, try again next iteration
		}
		_moved_to_current_goal = false;
		_local_goal_obsolete = false;
		_updating_local_goal = false;
		_local_goal_updated = true;
		determineNextNodeInPath();
		ROS_INFO_STREAM("Current goal node set to " << _current_goal_node);
	}
}

void GraphConstructor::determineNextNodeInPath() {
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

void GraphConstructor::publishExplorationGoalObsolete(bool obsolete) {
	rsb_nbv_exploration_msgs::ExplorationGoalObsolete msg;
	if (obsolete
			|| (_pursuing_local_goal
					&& ((!_node_comparator->isEmpty()
							&& _node_comparator->getBestNode()
									!= _current_goal_node)
							|| _local_goal_obsolete))
			|| (_pursuing_global_goal && !_reached_frontier_goal
					&& _current_goal_node != -1)) {
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
				!= rsb_nbv_exploration_msgs::Node::EXPLORED
				&& _rrg.nodes[iterator.first].status
						!= rsb_nbv_exploration_msgs::Node::FAILED
				&& _rrg.nodes[iterator.first].status
						!= rsb_nbv_exploration_msgs::Node::INACTIVE
				&& std::isinf(_rrg.nodes[iterator.first].cost_function) == 0) {
			_nodes_to_update.push_back(iterator.first);
		}
		if (_rrg.nodes[iterator.first].status
				!= rsb_nbv_exploration_msgs::Node::FAILED
				&& _rrg.nodes[iterator.first].status
						!= rsb_nbv_exploration_msgs::Node::INACTIVE
				&& _rrg.nodes[iterator.first].retry_inflation) { // retry previously failed inflation
			std::vector<int> engulfed_nodes =
					_collision_checker->inflateExistingNode(_rrg,
							iterator.first, _robot_pose, _nodes_to_update,
							_sort_nodes_to_update);
			if (engulfed_nodes.size() > 0)
				pruneEngulfedNodes(engulfed_nodes);
			_graph_searcher->rebuildIndex(_rrg);
			if (_global_exploration_active
					&& !_rrg.nodes.at(iterator.first).retry_inflation) // only prune if inflation is finished for this node
				_global_graph_handler->pruneFrontiersAndPathsAroundNewNode(_rrg,
						iterator.first);
		}
	}
}

void GraphConstructor::pruneEngulfedNodes(std::vector<int> engulfed_nodes) {
	std::set<int> pruned_nodes;
	std::set<int> pruned_edges;
	for (auto engulfed_node : engulfed_nodes) {
		if (_rrg.nodes.at(engulfed_node).connected_to.empty()
				|| _rrg.nearest_node != engulfed_node) { //don't prune the currently nearest node with connected paths, cannot be rerouted
			if (_rrg.nodes.at(engulfed_node).status
					== rsb_nbv_exploration_msgs::Node::FAILED
					&& !_rrg.nodes.at(engulfed_node).connected_to.empty()) { //node must be collision free to be engulfed, recover it
				_rrg.nodes[engulfed_node].status =
						rsb_nbv_exploration_msgs::Node::INITIAL;
				_collision_checker->findBestConnectionForNode(_rrg,
						_rrg.nodes[engulfed_node], _robot_pose, false);
			}
			pruned_nodes.insert(engulfed_node);
			_rrg.nodes.at(engulfed_node).status =
					rsb_nbv_exploration_msgs::Node::INACTIVE;
			for (int edge : _rrg.nodes.at(engulfed_node).edges) {
				_rrg.edges[edge].inactive = true;
				pruned_edges.insert(edge);
			}
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
	if (_rrg.nodes.at(node_one).distance_to_robot
			< _rrg.nodes.at(node_two).distance_to_robot) {
		return true;
	} else if (_rrg.nodes.at(node_one).distance_to_robot
			> _rrg.nodes.at(node_two).distance_to_robot) {
		return false;
	} else {
		return node_one < node_two;
	}
}

void GraphConstructor::publishNodeToUpdate() {
	if (!_nodes_to_update.empty()) {
		if (_sort_nodes_to_update)
			sortNodesToUpdateByDistanceToRobot();
		rsb_nbv_exploration_msgs::NodeToUpdate msg;
		msg.node = _rrg.nodes[_nodes_to_update.front()];
		msg.force_update = _nodes_to_update.front() == _last_goal_node;
		_node_to_update_publisher.publish(msg);
	} else if (_reupdate_nodes && !_nodes_to_reupdate.empty()) { // if list is empty, re-update nodes ordered ascending by their distance to the robot
		rsb_nbv_exploration_msgs::NodeToUpdate msg;
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
	case rsb_nbv_exploration_msgs::Node::EXPLORED:
		ROS_INFO_STREAM("RNE goal " << _current_goal_node << " explored");
		_node_comparator->removeNode(_current_goal_node);
		update_center = _rrg.nodes[_current_goal_node].position;
		updateNodes(update_center);
		resetNextNodeInPath();
		tryFailedNodesRecovery();
		_consecutive_failed_goals = 0;
		break;
	case rsb_nbv_exploration_msgs::Node::VISITED:
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
	case rsb_nbv_exploration_msgs::Node::ABORTED:
		ROS_INFO_STREAM("RNE goal " << _current_goal_node << " aborted");
		_rrg.nodes[_current_goal_node].status =
				_rrg.nodes[_current_goal_node].gain > 0 ?
						rsb_nbv_exploration_msgs::Node::INITIAL :
						rsb_nbv_exploration_msgs::Node::EXPLORED;
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
		_local_goal_obsolete = false;
		resetNextNodeInPath();
		break;
	case rsb_nbv_exploration_msgs::Node::FAILED:
		if (_rrg.nearest_node == _current_goal_node) {
			ROS_INFO_STREAM(
					"RNE goal " << _current_goal_node << " nearly visited, failed to approach closer");
			_rrg.nodes[_current_goal_node].status =
					rsb_nbv_exploration_msgs::Node::VISITED;
		} else {
			ROS_INFO_STREAM("RNE goal " << _current_goal_node << " failed");
			_failed_nodes_to_recover.push_back(_current_goal_node);
			if (++_consecutive_failed_goals >= _max_consecutive_failed_goals) {
				ROS_WARN_STREAM("Exploration aborted, robot stuck");
				_rrg.node_counter = -1; // for evaluation purposes
				stopExploration();
			}
		}
		_node_comparator->removeNode(_current_goal_node);
		_rrg.nodes[_current_goal_node].gain = 0;
		_rrg.nodes[_current_goal_node].heading_change_to_robot_best_view =
				_rrg.nodes[_current_goal_node].heading_change_to_robot;
		_rrg.nodes[_current_goal_node].reward_function = 0;

		if (_moved_to_current_goal) {
			update_center = _last_robot_pos;
			updateNodes(update_center);
		}
		tryFailedNodesRecovery();
		resetNextNodeInPath();
		break;
	case rsb_nbv_exploration_msgs::Node::INACTIVE:
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
		const rsb_nbv_exploration_msgs::Node::ConstPtr &updated_node) {
	if (!_local_running) { // discard updated node if local exploration is not running
		return;
	}
	if (updated_node->index >= _rrg.node_counter
			|| _rrg.nodes.at(updated_node->index).status
					== rsb_nbv_exploration_msgs::Node::INACTIVE) { //check if node was removed while being updated
		ROS_WARN_STREAM(
				"updated node " << updated_node->index << " became inactive while updating!");
		removeNodeFromUpdateLists(updated_node->index);
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
		removeNodeFromUpdateLists(updated_node->index);
		if (updated_node->status != rsb_nbv_exploration_msgs::Node::EXPLORED
				&& updated_node->status
						!= rsb_nbv_exploration_msgs::Node::FAILED
				&& updated_node->status
						!= rsb_nbv_exploration_msgs::Node::INACTIVE) {
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
			if (updated_node->status == rsb_nbv_exploration_msgs::Node::EXPLORED
					&& _current_goal_node == updated_node->index
					&& _node_comparator->isEmpty()) { // current goal is explored, change goal
				ROS_INFO_STREAM(
						"Current local goal node is already explored, became obsolete");
				_local_goal_obsolete = true;
			}
		}
		if (_global_exploration_active && check_waypoints) { // node was just added
			std::vector<geometry_msgs::Point> frontier_viewpoints =
					_global_graph_handler->pruneFrontiersAndPathsAroundNewNode(
							_rrg, updated_node->index); // if neighbor of new node is connected to frontiers, try to rewire paths
			_new_node_positions.insert(_new_node_positions.end(),
					frontier_viewpoints.begin(), frontier_viewpoints.end());
		}
		publishNodeToUpdate(); // if gain calculation is faster than update frequency, this needs to be called
	}
}

void GraphConstructor::tryFailedNodesRecovery() {
	if (!_failed_nodes_to_recover.empty()) {
		_failed_nodes_to_recover.erase(
				std::remove_if(_failed_nodes_to_recover.begin(),
						_failed_nodes_to_recover.end(),
						[this](int node) {
							int collision =
									_collision_checker->collisionCheckForFailedNode(
											_rrg, node, true);
							if (collision
									== CollisionChecker::Collisions::unknown) {
								return false; // keep node and try to recover again
							} else if (collision
									== CollisionChecker::Collisions::empty) {
								_rrg.nodes[node].status =
										rsb_nbv_exploration_msgs::Node::INITIAL;
								_collision_checker->findBestConnectionForNode(
										_rrg, _rrg.nodes[node], _robot_pose,
										false);
								_graph_path_calculator->updatePathsToRobot(node,
										_rrg, _robot_pose, false,
										_nodes_to_update,
										_sort_nodes_to_update); // check if recovered connection could improve other distances
								if (std::isinf(_rrg.nodes[node].cost_function)
										== 0) { // check if node is reachable
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
}

void GraphConstructor::addNodeToLastThreeNodesPath(int node) {
	if (!_reupdate_nodes)
		return;
	auto it = std::find(_last_three_nodes_path.begin(),
			_last_three_nodes_path.end(), node); //check if node is already in list
	if (it == _last_three_nodes_path.end()) {
		if (_last_three_nodes_path.size() >= 3) {
			_last_three_nodes_path.erase(_last_three_nodes_path.begin());
		}
		_last_three_nodes_path.push_back(node);
		_nodes_to_reupdate.clear();
		_nodes_to_reupdate = _node_comparator->getListOfNodes();
		std::sort(_nodes_to_reupdate.begin(), _nodes_to_reupdate.end(),
				[this](const int node_one, const int node_two) {
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
	ROS_INFO_STREAM("Start global exploration");
	if (_pursuing_local_goal) { // if a current frontier goal is active, it became obsolete
		ROS_INFO_STREAM(
				"Local goal became obsolete because of switch to global exploration");
		publishExplorationGoalObsolete(true);
		_current_goal_node = -1;
	}
	_local_running = false;
}

void GraphConstructor::localExplorationFinishedTimerCallback(
		const ros::TimerEvent &event) {
	_local_exploration_finished_timer.stop();
	if (_global_exploration_active) {
		ROS_INFO_STREAM(
				"Local exploration without goals, switch to global exploration");
		switchFromLocalToGlobalExploration();
	} else {
		ROS_INFO_STREAM("Exploration finished");
		_running = false;
	}
}

void GraphConstructor::switchFromGlobalToToLocalExploration(
		const geometry_msgs::Point &frontier,
		const std::vector<int> &connected_paths) {
	ROS_INFO_STREAM("Init local exploration");
	initLocalGraph(frontier);
	resetHelperClasses();
	_rrg.nodes.front().connected_to = connected_paths;
	_local_running = true;
	_local_exploration_finished_timer.start();
	ROS_INFO_STREAM("Init local exploration finished");
}

bool GraphConstructor::requestGoal(
		rsb_nbv_exploration_msgs::RequestGoal::Request &req,
		rsb_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.exploration_finished = !_running;
	if (_local_running) { // local goal
		res.goal_available = _local_goal_updated && _current_goal_node != -1;
		if (_current_goal_node != -1) {
			if (_rrg.nodes[_current_goal_node].status
					== rsb_nbv_exploration_msgs::Node::VISITED) {
				_rrg.nodes[_current_goal_node].status =
						rsb_nbv_exploration_msgs::Node::ACTIVE_VISITED;
			} else if (_rrg.nodes[_current_goal_node].status
					== rsb_nbv_exploration_msgs::Node::INITIAL) {
				_rrg.nodes[_current_goal_node].status =
						rsb_nbv_exploration_msgs::Node::ACTIVE;
			}
			res.goal = _rrg.nodes.at(_current_goal_node).position;
			res.best_yaw = _graph_path_calculator->determineGoalYaw(
					_current_goal_node, _rrg, _last_robot_pos, !_running);
			_pursuing_local_goal = true;
			_pursuing_global_goal = false;
			_local_goal_updated = false;
			ROS_INFO_STREAM(
					"Request goal: local node " << _current_goal_node << " at (" << _rrg.nodes.at(_current_goal_node).position.x << ", " << _rrg.nodes.at(_current_goal_node).position.y << ")");
		}
	} else if (_global_exploration_active && !_local_running) { // global goal
		geometry_msgs::Point goal;
		double yaw;
		if (_global_graph_handler->getFrontierGoal(goal, yaw,
				_robot_pose.position)) {
			res.goal_available = true;
			res.goal = goal;
			res.best_yaw = yaw;
			_pursuing_global_goal = true;
			_pursuing_local_goal = false;
			_global_goal_updated = false;
			_reached_frontier_goal = false;
			ROS_INFO_STREAM(
					"Request goal: global target at (" << goal.x << ", " << goal.y << ")");
		} else {
			res.goal_available = false;
		}
	}
	return true;
}

bool GraphConstructor::requestPath(
		rsb_nbv_exploration_msgs::RequestPath::Request &req,
		rsb_nbv_exploration_msgs::RequestPath::Response &res) {
	if (_local_running) { // local path
		if (_current_goal_node != -1) {
			ROS_INFO_STREAM("Get path to local node " << _current_goal_node);
			std::vector<geometry_msgs::PoseStamped> rrg_path;
			_graph_path_calculator->getLocalNavigationPath(rrg_path, _rrg,
					_current_goal_node, _robot_pose.position);
			res.path = rrg_path;
			return true;
		}
	} else if (_global_exploration_active && !_local_running) { // global path
		std::vector<geometry_msgs::PoseStamped> path;
		if (_global_graph_handler->getFrontierPath(path,
				_robot_pose.position)) {
			res.path = path;
			return true;
		}
	}
	return false;
}

bool GraphConstructor::updateCurrentGoal(
		rsb_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
		rsb_nbv_exploration_msgs::UpdateCurrentGoal::Response &res) {
	if (_pursuing_local_goal) { // local goal
		ROS_INFO_STREAM(
				"Updating current local goal node " << _current_goal_node << " updating local: " << _updating_local_goal);
		if (_current_goal_node != -1 && !_updating_local_goal) {
			_pursuing_local_goal = false;
			_updating_local_goal = true;
			_rrg.nodes[_current_goal_node].status = req.status;
			handleCurrentLocalGoalFinished();
		}
	} else if (_pursuing_global_goal) { // global goal
		ROS_INFO_STREAM(
				"Updating global goal target, updating global: " << _updating_global_goal);
		if (!_updating_global_goal) {
			_pursuing_global_goal = false;
			_updating_global_goal = true;
			if (req.status == rsb_nbv_exploration_msgs::Node::VISITED
					|| req.status == rsb_nbv_exploration_msgs::Node::ABORTED) { // reached frontier goal or aborted it for new one
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
				if (_global_graph_handler->targetFailed()) {
					ROS_INFO_STREAM("Exploration finished");
					_running = false;
				}
			}
		}
	} else {
		ROS_INFO_STREAM("Currently not pursuing any goal, update discarded");
	}
	res.success = true;
	res.message = "Changed current goal's status";
	return true;
}

void GraphConstructor::startExploration() {
	initExploration(_graph_path_calculator->getRobotPose());
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
	initExploration(geometry_msgs::Pose());
	res.message = true;
	res.message = "Tree reset";
	return true;
}

void GraphConstructor::dynamicReconfigureCallback(
		rsb_nbv_exploration::GraphConstructorConfig &config, uint32_t level) {
	_local_graph_radius = std::max(config.local_graph_radius,
			2 * _robot_radius); // cannot fall below the robot diameter
	_local_graph_pruning_radius_squared = pow(
			_local_graph_radius + _robot_radius, 2); // add buffer to pruning
	_local_sampling_radius = std::min(_local_graph_radius,
			config.local_sampling_radius); // cannot exceed local graph radius
	_graph_path_calculator->dynamicReconfigureCallback(config, level);
	if (_global_exploration_active) {
		_global_graph_handler->dynamicReconfigureCallback(config, level);
	}
}

}
