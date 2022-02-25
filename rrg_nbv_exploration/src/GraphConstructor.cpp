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
	private_nh.param("sensor_height", _sensor_height, 0.5);
	double min_edge_distance;
	private_nh.param("min_edge_distance", _min_edge_distance, 1.0);
	private_nh.param("max_edge_distance", _max_edge_distance, 2.0);
	_max_edge_distance_squared = pow(_max_edge_distance, 2);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	_robot_radius_squared = pow(_robot_radius, 2);
	private_nh.param("grid_map_resolution", _grid_map_resolution, 0.05);
	private_nh.param("local_sampling_radius", _local_sampling_radius, 5.0);
	private_nh.param("exploration_finished_timer_duration",
			_exploration_finished_timer_duration, 10.0);
	double nearest_node_tolerance;
	private_nh.param("nearest_node_tolerance", nearest_node_tolerance, 0.1);
	private_nh.param("max_consecutive_failed_goals",
			_max_consecutive_failed_goals, 5);
	private_nh.param("auto_homing", _auto_homing, false);
	private_nh.param("reupdate_nodes", _reupdate_nodes, true);

	ros::NodeHandle nh("rne");
	_rrg_publisher = nh.advertise<rrg_nbv_exploration_msgs::Graph>("rrg", 1);
	_node_to_update_publisher = nh.advertise<
			rrg_nbv_exploration_msgs::NodeToUpdate>("node_to_update", 1);
	_updated_node_subscriber = nh.subscribe("updated_node", 1,
			&GraphConstructor::updatedNodeCallback, this);
	_best_and_current_goal_publisher = nh.advertise<
			rrg_nbv_exploration_msgs::BestAndCurrentNode>("bestAndCurrentGoal",
			1);
	_request_goal_service = nh.advertiseService("requestGoal",
			&GraphConstructor::requestGoal, this);
	_request_path_service = nh.advertiseService("requestPath",
			&GraphConstructor::requestPath, this);
	_update_current_goal_service = nh.advertiseService("updateCurrentGoal",
			&GraphConstructor::updateCurrentGoal, this);

	_exploration_finished_timer = _nh.createTimer(
			ros::Duration(_exploration_finished_timer_duration),
			&GraphConstructor::explorationFinishedTimerCallback, this, false,
			false);

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

	_graph_searcher.reset(new GraphSearcher());
	_collision_checker.reset(new CollisionChecker());
	_graph_path_calculator.reset(new GraphPathCalculator());
	_node_comparator.reset(new NodeComparator());
	_running = false;
	_explored_current_goal_node_by_update = false;
}

void GraphConstructor::initRrg(const geometry_msgs::Point &seed) {
	_rrg.nodes.clear();
	_rrg.edges.clear();
	_nodes_to_update.clear();
	_rrg.header.frame_id = "/map";
	_rrg.ns = "rrt_tree";
	_rrg.node_counter = 0;
	_rrg.edge_counter = 0;
	rrg_nbv_exploration_msgs::Node root;
	root.position = seed;
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
	_goal_updated = true;
	_updating = false;
	_sort_nodes_to_update = false;
	_last_robot_pos = seed;
	_consecutive_failed_goals = 0;
	_graph_searcher->initialize(_rrg);
	_nodes_to_update.push_back(0);
	_last_three_nodes_path.push_back(0);
	_node_comparator->initialization();
	_generator.seed(time(NULL));
	_collision_checker->initialize(_rrg, _graph_searcher,
			_graph_path_calculator);
}

void GraphConstructor::startRrgConstruction() {
	initRrg(_graph_path_calculator->getRobotPose().position);
	_running = true;
}

void GraphConstructor::stopRrgConstruction() {
	_running = false;
}

void GraphConstructor::runRrgConstruction() {
	_rrg.header.stamp = ros::Time::now();
	if (_running && _map_min_bounding[0] && _map_min_bounding[1]
			&& _map_min_bounding[2]) {
		_robot_pose = _graph_path_calculator->getRobotPose();
		//TODO: get next_node and edge_to_next_node, reset them on goal change and when nearest node changes
		bool updatePathsWithReset = determineNearestNodeToRobot(
				_robot_pose.position); //check if nearest node to robot changed which means robot moved
		expandGraph(false, !updatePathsWithReset, _robot_pose); //global expansion
		if (_local_sampling_radius > 0) //local sampling expansion
			expandGraph(true, !updatePathsWithReset, _robot_pose);
		if (updatePathsWithReset) { //robot moved, update paths
			_graph_path_calculator->updatePathsToRobot(_rrg.nearest_node, _rrg,
					_robot_pose, true, _nodes_to_update, _sort_nodes_to_update);
			determineNextNodeInPath();
		} else { //check if robot heading changed and update heading
			if (_graph_path_calculator->updateHeadingToRobot(_rrg.nearest_node,
					_rrg, _robot_pose, _next_node, _edge_to_next_node,
					_distance_to_nearest_node_squared, _nodes_to_update,
					_sort_nodes_to_update))
				_node_comparator->robotMoved();
		}
		_node_comparator->maintainList(_rrg);
		checkCurrentGoal();
		publishBestAndCurrentNode();
		publishNodeToUpdate();
		if (_nodes_to_update.empty() && _current_goal_node == -1) {
			_exploration_finished_timer.start();
		}
	}
	_rrg_publisher.publish(_rrg);
}

void GraphConstructor::expandGraph(bool local, bool updatePaths,
		geometry_msgs::Pose robot_pos) {
	geometry_msgs::Point rand_sample;
	if (local)
		samplePointLocally(rand_sample, _last_robot_pos);
	else
		samplePoint(rand_sample);
	rrg_nbv_exploration_msgs::Node node;
	if (_collision_checker->steer(_rrg, node, rand_sample, robot_pos)) {
		if (!std::isinf(node.cost_function)) { //do not update unreachable nodes
			if (updatePaths)
				_graph_path_calculator->updatePathsToRobot(node.index, _rrg,
						robot_pos, false, _nodes_to_update,
						_sort_nodes_to_update); //check if new connection could improve other distances
			_nodes_to_update.push_back(node.index);
			_sort_nodes_to_update = true;
		}
		_graph_searcher->rebuildIndex(_rrg);
		_exploration_finished_timer.stop();
	}
}

void GraphConstructor::samplePoint(geometry_msgs::Point &rand_sample) {
	std::uniform_real_distribution<double> x_distribution(_map_min_bounding[0],
			_map_max_bounding[0]);
	std::uniform_real_distribution<double> y_distribution(_map_min_bounding[1],
			_map_max_bounding[1]);
	rand_sample.x = x_distribution(_generator);
	rand_sample.y = y_distribution(_generator);
}

bool GraphConstructor::samplePointLocally(geometry_msgs::Point &rand_sample,
		geometry_msgs::Point center) {
	std::uniform_real_distribution<double> r_distribution(0, 1);
	std::uniform_real_distribution<double> theta_distribution(0, 2 * M_PI);
	double r = _local_sampling_radius * sqrt(r_distribution(_generator));
	double theta = theta_distribution(_generator);
	rand_sample.x = center.x + r * cos(theta);
	rand_sample.y = center.y + r * sin(theta);
	return (rand_sample.x >= _map_min_bounding[0]
			&& rand_sample.x <= _map_max_bounding[0]
			&& rand_sample.y >= _map_min_bounding[1]
			&& rand_sample.y <= _map_max_bounding[1]);
}

void GraphConstructor::checkCurrentGoal() {
	if (_current_goal_node == -1 && !_node_comparator->isEmpty()) {
		_current_goal_node = _node_comparator->getBestNode();
		_moved_to_current_goal = false;
		_goal_updated = true;
		_updating = false;
		determineNextNodeInPath();
		ROS_INFO_STREAM("Current goal node set to " << _current_goal_node);
	}
}

void GraphConstructor::determineNextNodeInPath() {
	if (_current_goal_node >= 0
			&& _rrg.nodes[_current_goal_node].path_to_robot.size() > 1) {
		_next_node = _rrg.nodes[_current_goal_node].path_to_robot.at(1); //next goal on the path to the robot
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
		if (nearest_node != _rrg.nearest_node) { //if nearest node changed
			if (_next_node != -1 && nearest_node != _next_node) {
				//projection of robot pos on the edge between nearest and next node
				double projection_on_edge =
						((pos.x - _rrg.nodes[_rrg.nearest_node].position.x)
								* (_rrg.nodes[_next_node].position.x
										- _rrg.nodes[_rrg.nearest_node].position.x)
								+ (pos.y
										- _rrg.nodes[_rrg.nearest_node].position.y)
										* (_rrg.nodes[_next_node].position.y
												- _rrg.nodes[_rrg.nearest_node].position.y))
								/ pow(_rrg.edges[_edge_to_next_node].length, 2);
				if (projection_on_edge > 0 && projection_on_edge < 1) { //if projection is outside these bounds, the robot is outside the line segment
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
					if (distance_to_edge <= min_distance) { //robot is closer to edge than to nearest node, keep current one
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
		} else { //nearest node remained the same, just update distance
			double recalculate_distance = _robot_radius_squared / SQRT10;
			if (nearest_node == _current_goal_node
					&& _distance_to_nearest_node_squared > recalculate_distance
					&& min_distance <= recalculate_distance) { //update current goal node gain when 1/10 robot radius away
				_nodes_to_reupdate.clear();
				_nodes_to_reupdate.push_back(_node_comparator->getBestNode());
			}
			_distance_to_nearest_node_squared = min_distance;
		}
	}
	return false;
}

void GraphConstructor::publishBestAndCurrentNode() {
	rrg_nbv_exploration_msgs::BestAndCurrentNode msg;
	msg.current_goal = _current_goal_node;
	msg.best_node =
			_node_comparator->isEmpty() ?
					(_explored_current_goal_node_by_update ?
							-1 : _current_goal_node) :
					_node_comparator->getBestNode(); //set best node to -1 if current goal was explored and no unexplored nodes are available
	msg.goal_updated = _goal_updated;
	_best_and_current_goal_publisher.publish(msg);
}

void GraphConstructor::updateNodes(geometry_msgs::Point center_node) {
	std::vector<std::pair<int, double>> updatable_nodes =
			_graph_searcher->searchInRadius(center_node, _radius_search_range);
	for (auto iterator : updatable_nodes) {
		if (_rrg.nodes[iterator.first].status
				!= rrg_nbv_exploration_msgs::Node::EXPLORED
				&& _rrg.nodes[iterator.first].status
						!= rrg_nbv_exploration_msgs::Node::FAILED
				&& !std::isinf(_rrg.nodes[iterator.first].cost_function)) {
			_nodes_to_update.push_back(iterator.first);
		}
		if (_rrg.nodes[iterator.first].status
				!= rrg_nbv_exploration_msgs::Node::FAILED
				&& _rrg.nodes[iterator.first].retry_inflation) //retry previously failed inflation
			_collision_checker->inflateExistingNode(_rrg, iterator.first,
					_robot_pose, _nodes_to_update, _sort_nodes_to_update);
	}
}

void GraphConstructor::sortNodesToUpdateByDistanceToRobot() {
	_nodes_to_update.sort([this](int node_one, int node_two) {
		return compareNodeDistancesToRobot(node_one, node_two);
	});
	_sort_nodes_to_update = false;

}

bool GraphConstructor::compareNodeDistancesToRobot(const int &node_one,
		const int &node_two) {
	return _rrg.nodes[node_one].distance_to_robot
			<= _rrg.nodes[node_two].distance_to_robot;
}

void GraphConstructor::publishNodeToUpdate() {
	if (!_nodes_to_update.empty()) {
		if (_sort_nodes_to_update)
			sortNodesToUpdateByDistanceToRobot();
		rrg_nbv_exploration_msgs::NodeToUpdate msg;
		msg.node = _rrg.nodes[_nodes_to_update.front()];
		msg.force_update = _nodes_to_update.front() == _last_goal_node;
		_node_to_update_publisher.publish(msg);
	} else if (_reupdate_nodes && !_nodes_to_reupdate.empty()) { //if list is empty, re-update nodes ordered ascending by their distance to the robot
		rrg_nbv_exploration_msgs::NodeToUpdate msg;
		int node = _nodes_to_reupdate.front();
		_nodes_to_reupdate.erase(_nodes_to_reupdate.begin());
		if (_rrg.nodes[node].distance_to_robot > _sensor_range) { //don't re-update nodes outside of sensor range
			_nodes_to_reupdate.clear();
			return;
		}
		msg.node = _rrg.nodes[node];
		msg.force_update = false;
		_node_to_update_publisher.publish(msg);
	}
}

void GraphConstructor::handleCurrentGoalFinished() {
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
		if (_moved_to_current_goal) { //do not update nodes because of constant re-updating
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
			_rrg.node_counter = -1; //for evaluation purposes
			stopRrgConstruction();
		}
		if (_moved_to_current_goal) {
			update_center = _last_robot_pos;
			updateNodes(update_center);
		}
		resetNextNodeInPath();
		break;
	default:
		//active or waiting (should not occur)
		ROS_INFO_STREAM(
				"RNE goal " << _current_goal_node << " active or waiting");
		return;
	}
	_last_goal_node = _current_goal_node;
	_current_goal_node = -1;
}

void GraphConstructor::updatedNodeCallback(
		const rrg_nbv_exploration_msgs::Node::ConstPtr &updated_node) {
	if (updated_node->index != _last_updated_node
			|| _rrg.nodes[_last_updated_node].gain != updated_node->gain
			|| _rrg.nodes[_last_updated_node].best_yaw != updated_node->best_yaw
			|| _rrg.nodes[_last_updated_node].status != updated_node->status) {
		_rrg.nodes[updated_node->index].gain = updated_node->gain;
		_rrg.nodes[updated_node->index].best_yaw = updated_node->best_yaw;
		_rrg.nodes[updated_node->index].status = updated_node->status;
		_rrg.nodes[updated_node->index].position.z = updated_node->position.z;
		_last_updated_node = updated_node->index;
		_nodes_to_update.remove(updated_node->index);
		_node_comparator->removeNode(updated_node->index);
		if (updated_node->status != rrg_nbv_exploration_msgs::Node::EXPLORED
				&& updated_node->status
						!= rrg_nbv_exploration_msgs::Node::FAILED) {
			_rrg.nodes[updated_node->index].heading_change_to_robot_best_view =
					_graph_path_calculator->calculateHeadingChangeToBestView(
							_rrg.nodes[updated_node->index]);
			_rrg.nodes[updated_node->index].cost_function =
					_graph_path_calculator->calculateCostFunction(
							_rrg.nodes[updated_node->index]);
			_node_comparator->addNode(updated_node->index);
			_sort_nodes_to_update = true;
		} else {
			_rrg.nodes[updated_node->index].gain = 0;
			_rrg.nodes[updated_node->index].heading_change_to_robot_best_view =
					_rrg.nodes[updated_node->index].heading_change_to_robot;
			_rrg.nodes[updated_node->index].reward_function = 0.0;
			if (updated_node->status == rrg_nbv_exploration_msgs::Node::EXPLORED
					&& _current_goal_node == updated_node->index
					&& _node_comparator->isEmpty()) { //current goal is explored, change goal
				_explored_current_goal_node_by_update = true;
			}
		}
		publishNodeToUpdate(); //if gain calculation is faster than update frequency, this needs to be called
	}
}

void GraphConstructor::tryFailedNodesRecovery() {
	_failed_nodes_to_recover.erase(
			std::remove_if(_failed_nodes_to_recover.begin(),
					_failed_nodes_to_recover.end(),
					[this](int node) {
						int collision =
								_collision_checker->collisionCheckForFailedNode(
										_rrg, node);
						if (collision == Collisions::unknown) {
							return false; //keep node and try to recover again
						} else if (collision == Collisions::empty) {
							_rrg.nodes[node].status =
									rrg_nbv_exploration_msgs::Node::INITIAL;
							_graph_path_calculator->findBestConnectionForNode(
									_rrg, _rrg.nodes[node], _robot_pose, false);
							_graph_path_calculator->updatePathsToRobot(node,
									_rrg, _robot_pose, false, _nodes_to_update,
									_sort_nodes_to_update); //check if recovered connection could improve other distances
							if (!std::isinf(_rrg.nodes[node].cost_function)) { //check if node is reachable
								_nodes_to_update.push_back(node); //recalculate node gain
								_sort_nodes_to_update = true;
							}
						}
						return true;
					}),_failed_nodes_to_recover.end());
	_collision_checker->retryEdges(_rrg, _robot_pose, _nodes_to_update,
			_sort_nodes_to_update); //try to rebuild failed edges
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

void GraphConstructor::explorationFinishedTimerCallback(
		const ros::TimerEvent &event) {
	ROS_INFO_STREAM("Exploration finished");
	_running = false;
	_exploration_finished_timer.stop();
	if (_auto_homing) {
		ROS_INFO_STREAM("Return to start");
		_current_goal_node = 0; //go back to root node
		_goal_updated = true;
	}
}

bool GraphConstructor::requestGoal(
		rrg_nbv_exploration_msgs::RequestGoal::Request &req,
		rrg_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.goal_available = _goal_updated && _current_goal_node != -1;
	res.exploration_finished = !_running
			&& (!_auto_homing || _rrg.nearest_node == 0);
	if (res.goal_available) {
		if (_rrg.nodes[_current_goal_node].status
				== rrg_nbv_exploration_msgs::Node::VISITED) {
			_rrg.nodes[_current_goal_node].status =
					rrg_nbv_exploration_msgs::Node::ACTIVE_VISITED;
		} else if (_rrg.nodes[_current_goal_node].status
				== rrg_nbv_exploration_msgs::Node::INITIAL) {
			_rrg.nodes[_current_goal_node].status =
					rrg_nbv_exploration_msgs::Node::ACTIVE;
		}
		res.goal = _rrg.nodes[_current_goal_node].position;
		res.best_yaw = _graph_path_calculator->determineGoalYaw(
				_current_goal_node, _rrg, _last_robot_pos, !_running);
		_goal_updated = false;
	}
	return true;
}

bool GraphConstructor::requestPath(
		rrg_nbv_exploration_msgs::RequestPath::Request &req,
		rrg_nbv_exploration_msgs::RequestPath::Response &res) {
	if (_current_goal_node != -1) {
		std::vector<geometry_msgs::PoseStamped> rrt_path;
		_graph_path_calculator->getNavigationPath(rrt_path, _rrg,
				_current_goal_node, _last_robot_pos);
		res.path = rrt_path;
		return true;
	}
	return false;
}

bool GraphConstructor::updateCurrentGoal(
		rrg_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
		rrg_nbv_exploration_msgs::UpdateCurrentGoal::Response &res) {
	if (!_running)
		_auto_homing = false; //end exploration when navigation to home node is finished (even if it failed)
	if (_current_goal_node != -1 && !_updating) {
		_updating = true;
		_rrg.nodes[_current_goal_node].status = req.status;
		handleCurrentGoalFinished();
	}
	res.success = true;
	res.message = "Changed current goal's status";
	return true;
}

bool GraphConstructor::setRrgState(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (req.data) {
		if (_running) {
			res.success = false;
			res.message = "Tree construction already running";
		} else {
			startRrgConstruction();
			res.success = true;
			res.message = "Tree construction started";
		}
	} else {
		if (!_running) {
			res.success = false;
			res.message = "Tree construction already stopped";
		} else {
			stopRrgConstruction();
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
	initRrg(geometry_msgs::Point());
	res.message = true;
	res.message = "Tree reset";
	return true;
}

void GraphConstructor::dynamicReconfigureCallback(
		rrg_nbv_exploration::GraphConstructorConfig &config, uint32_t level) {
	_node_comparator->dynamicReconfigureCallback(config, level);
	_graph_path_calculator->dynamicReconfigureCallback(config, level);
}

}
