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
	private_nh.param("robot_radius", _robot_radius, 1.0);
	_max_edge_distance_squared = pow(_robot_radius, 2);
	private_nh.param("grid_map_resolution", _grid_map_resolution, 0.05);
	private_nh.param("local_sampling_radius", _local_sampling_radius, 5.0);
	private_nh.param("exploration_finished_timer_duration",
			_exploration_finished_timer_duration, 10.0);
	double nearest_node_tolerance;
	private_nh.param("nearest_node_tolerance", nearest_node_tolerance, 0.1);
	_nearest_node_tolerance_squared = pow(nearest_node_tolerance, 2);

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
}

bool GraphConstructor::initRrg(const geometry_msgs::Point &seed) {
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
	root.index = 0;
	root.distanceToRobot = 0;
	root.pathToRobot.push_back(0);
	root.radius = _robot_radius;
	root.squared_radius = pow(_robot_radius, 2);
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
	_graph_searcher->initialize(_rrg);
	_nodes_to_update.push_back(0);
	_node_comparator->initialization();
	_generator.seed(time(NULL));
	return _collision_checker->initialize(seed);
}

void GraphConstructor::startRrgConstruction() {
	if (initRrg(_graph_path_calculator->getRobotPose().position)) {
		_running = true;
	} else {
		ROS_WARN_STREAM(
				"Unable to start RNE because of obstacles too close to the robot!");
	}
}

void GraphConstructor::stopRrgConstruction() {
	_running = false;
}

void GraphConstructor::runRrgConstruction() {
	_rrg.header.stamp = ros::Time::now();
	if (_running && _map_min_bounding[0] && _map_min_bounding[1]
			&& _map_min_bounding[2]) {
		bool updatePathsWithReset = determineNearestNodeToRobot();
		expandGraph(false, !updatePathsWithReset);
		if (_local_sampling_radius > 0)
			expandGraph(true, !updatePathsWithReset);
		if (updatePathsWithReset)
			_graph_path_calculator->updatePathsToRobot(_rrg.nearest_node, _rrg);
		_node_comparator->maintainList(_rrg);
		checkCurrentGoal();
		publishNodeWithBestGain();
		publishNodeToUpdate();
		if (_nodes_to_update.empty() && _current_goal_node == -1) {
			_exploration_finished_timer.start();
		}
	}
	_rrg_publisher.publish(_rrg);
}

void GraphConstructor::expandGraph(bool local, bool updatePaths) {
	geometry_msgs::Point rand_sample;
	if (local)
		samplePointLocally(rand_sample, _last_robot_pos);
	else
		samplePoint(rand_sample);
	double min_distance;
	int nearest_node;
	_graph_searcher->findNearestNeighbour(rand_sample, min_distance,
			nearest_node);
	if (min_distance >= _rrg.nodes[nearest_node].radius) {
		if (min_distance > _rrg.nodes[nearest_node].radius) {
			// if random sample is further away than max edge distance, replace it at max distance to the nearest node on a line with the sample
			double distance = sqrt(min_distance);
			rand_sample.x = _rrg.nodes[nearest_node].position.x
					- (_rrg.nodes[nearest_node].radius
							* (_rrg.nodes[nearest_node].position.x
									- rand_sample.x) / distance);
			rand_sample.y = _rrg.nodes[nearest_node].position.y
					- (_rrg.nodes[nearest_node].radius
							* (_rrg.nodes[nearest_node].position.y
									- rand_sample.y) / distance);
		}
		connectNewNode(rand_sample, nearest_node, updatePaths);
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

void GraphConstructor::alignPointToGridMap(geometry_msgs::Point &rand_sample) {
	rand_sample.x = (round(rand_sample.x / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
	rand_sample.y = (round(rand_sample.y / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
}

void GraphConstructor::connectNewNode(geometry_msgs::Point rand_sample,
		int nearest_node, bool updatePaths) {
	alignPointToGridMap(rand_sample);
	rrg_nbv_exploration_msgs::Node node;
//	ROS_INFO_STREAM(
//			"connect new node at " << rand_sample.x << "," << rand_sample.y << " with nn=" << nearest_node);
	std::vector<std::pair<int, double>> engulfing_nodes =
			_graph_searcher->searchInRadius(rand_sample,
					pow(_radius_search_range, 2));
	for (auto it : engulfing_nodes) { //check if another node's radius engulfs the newly sampled point, discard it then
		if (_rrg.nodes[it.first].squared_radius
				> it.second + _grid_map_resolution)
			return;
	}
	if (_collision_checker->steer(node, _rrg.nodes[nearest_node],
			rand_sample)) {
		ROS_INFO_STREAM("steer successful, radius: " << node.radius);
		node.status = rrg_nbv_exploration_msgs::Node::INITIAL;
		node.gain = -1;
		node.index = _rrg.node_counter;
		std::vector<std::pair<int, double>> nodes =
				_graph_searcher->searchInRadius(node.position,
						pow(node.radius, 2));
		if (nodes.size() == 0) {
			ROS_INFO_STREAM("no nodes in radius, add nearest neighbor");
			nodes.push_back(
					std::make_pair(nearest_node,
							pow(_rrg.nodes[nearest_node].radius, 2)));
		}
		ROS_INFO_STREAM("number of nodes in radius: " << nodes.size());
		double height = 0;
		double shortest_distance = std::numeric_limits<double>::infinity();
		int node_with_shortest_distance = -1;
		for (auto it : nodes) {
			ROS_INFO_STREAM("add node " << it.first);
			double distance = sqrt(it.second);
			height += _rrg.nodes[it.first].position.z;
			rrg_nbv_exploration_msgs::Edge edge;
			edge.index = _rrg.edge_counter++;
			edge.first_node = it.first;
			edge.second_node = node.index;
			edge.length = distance;
			_rrg.edges.push_back(edge);
			node.edges.push_back(edge.index);
			node.edge_counter++;
			_rrg.nodes[it.first].edges.push_back(edge.index);
			_rrg.nodes[it.first].edge_counter++;
			//find shortest distance to new node
			double distance_to_robot = _rrg.nodes[it.first].distanceToRobot
					+ distance;
			if (distance_to_robot < shortest_distance) {
				shortest_distance = distance_to_robot;
				node_with_shortest_distance = it.first;
			}
		}
		ROS_INFO_STREAM("all nodes added ");
		node.position.z = height / nodes.size();
		node.distanceToRobot = shortest_distance;
		node.pathToRobot = _rrg.nodes[node_with_shortest_distance].pathToRobot;
		node.pathToRobot.push_back(node.index);
		_rrg.node_counter++;
		_rrg.nodes.push_back(node);
		if (updatePaths)
			_graph_path_calculator->updatePathsToRobot(node.index, _rrg, false); //check if new connection could improve other distances
		_nodes_to_update.push_back(node.index);
		_sort_nodes_to_update = true;
		_graph_searcher->rebuildIndex(_rrg);
		_exploration_finished_timer.stop();
	}
}

void GraphConstructor::checkCurrentGoal() {
	if (_current_goal_node == -1 && !_node_comparator->isEmpty()) {
		_current_goal_node = _node_comparator->getBestNode();
		_moved_to_current_goal = false;
		_goal_updated = true;
		_updating = false;
		ROS_INFO_STREAM("Current goal node set to " << _current_goal_node);
	}
}

bool GraphConstructor::determineNearestNodeToRobot() {
	geometry_msgs::Point pos = _graph_path_calculator->getRobotPose().position;
	if (pos.x != _last_robot_pos.x || pos.y != _last_robot_pos.y
			|| pos.z != _last_robot_pos.z) {
		_last_robot_pos = pos;
		double min_distance;
		int nearest_node;
		_graph_searcher->findNearestNeighbour(pos, min_distance, nearest_node);
		if (nearest_node != _rrg.nearest_node) { //if nearest node changed
			if (!_graph_path_calculator->neighbourNodes(_rrg, nearest_node,
					_rrg.nearest_node)) {
				//check if robot came off path and is close to a non-neighbor node
				double distance_squared = pow(
						_rrg.nodes[_rrg.nearest_node].position.x
								- _rrg.nodes[nearest_node].position.x, 2)
						+ pow(
								_rrg.nodes[_rrg.nearest_node].position.y
										- _rrg.nodes[nearest_node].position.y,
								2);
				if (distance_squared > _nearest_node_tolerance_squared) {
					return false;
				}
			}
			_moved_to_current_goal = true;
			_node_comparator->robotMoved();
			_rrg.nearest_node = nearest_node;
			return true;
		}
	}
	return false;
}

void GraphConstructor::publishNodeWithBestGain() {
	rrg_nbv_exploration_msgs::BestAndCurrentNode msg;
	msg.current_goal = _current_goal_node;
	msg.best_node =
			_node_comparator->isEmpty() ?
					_current_goal_node : _node_comparator->getBestNode();
	msg.goal_updated = _goal_updated;
	_best_and_current_goal_publisher.publish(msg);
}

void GraphConstructor::updateNodes(geometry_msgs::Point center_node) {
//ROS_INFO("start updating nodes");
	std::vector<std::pair<int, double>> updatable_nodes =
			_graph_searcher->searchInRadius(center_node, _radius_search_range);
	for (auto iterator : updatable_nodes) {
		if (_rrg.nodes[iterator.first].status
				!= rrg_nbv_exploration_msgs::Node::EXPLORED
				&& _rrg.nodes[iterator.first].status
						!= rrg_nbv_exploration_msgs::Node::FAILED) {
			_node_comparator->removeNode(iterator.first);
			_nodes_to_update.push_back(iterator.first);
			_sort_nodes_to_update = true;
			_rrg.nodes[iterator.first].gain = -1;
		}
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
	return _rrg.nodes[node_one].distanceToRobot
			<= _rrg.nodes[node_two].distanceToRobot;
}

void GraphConstructor::publishNodeToUpdate() {
	if (!_nodes_to_update.empty()) {
		if (_sort_nodes_to_update)
			sortNodesToUpdateByDistanceToRobot();
		rrg_nbv_exploration_msgs::NodeToUpdate msg;
		msg.node = _rrg.nodes[_nodes_to_update.front()];
		msg.force_update = _nodes_to_update.front() == _last_goal_node;
		_node_to_update_publisher.publish(msg);
	}
}

void GraphConstructor::updateCurrentGoal() {
//	ROS_INFO_STREAM(
//			"Update current goal " << _current_goal_node << " with status " << (int)_rrg.nodes[_current_goal_node].status);
	geometry_msgs::Point update_center;
	switch (_rrg.nodes[_current_goal_node].status) {
	case rrg_nbv_exploration_msgs::Node::EXPLORED:
		ROS_INFO("RNE goal explored");
		_node_comparator->removeNode(_current_goal_node);
		update_center = _rrg.nodes[_current_goal_node].position;
		updateNodes(update_center);
		break;
	case rrg_nbv_exploration_msgs::Node::VISITED:
		ROS_INFO("RNE goal visited");
		_node_comparator->removeNode(_current_goal_node);
		_sort_nodes_to_update = true;
		_rrg.nodes[_current_goal_node].gain = 0;
		update_center = _rrg.nodes[_current_goal_node].position;
		updateNodes(update_center);
		break;
	case rrg_nbv_exploration_msgs::Node::ABORTED:
		ROS_INFO("RNE goal aborted");
		_rrg.nodes[_current_goal_node].status =
				rrg_nbv_exploration_msgs::Node::INITIAL;
		if (_moved_to_current_goal) {
			ROS_INFO("update nodes");
			_node_comparator->removeNode(_current_goal_node);
			_sort_nodes_to_update = true;
			update_center = _last_robot_pos;
			updateNodes(update_center);
		}
		break;
	case rrg_nbv_exploration_msgs::Node::FAILED:
		ROS_INFO("RNE goal failed");
		_node_comparator->removeNode(_current_goal_node);
		_rrg.nodes[_current_goal_node].gain = 0;
		if (_moved_to_current_goal) {
			update_center = _last_robot_pos;
			updateNodes(update_center);
		}
		break;
	default:
		//active or waiting (should not occur)
		ROS_INFO("RNE goal active or waiting");
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
		if (updated_node->status != rrg_nbv_exploration_msgs::Node::EXPLORED
				&& updated_node->status
						!= rrg_nbv_exploration_msgs::Node::FAILED) {
			_node_comparator->addNode(updated_node->index);
		}
		publishNodeToUpdate(); //if gain calculation is faster than update frequency, this needs to be called
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
}

bool GraphConstructor::requestGoal(
		rrg_nbv_exploration_msgs::RequestGoal::Request &req,
		rrg_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.goal_available = _goal_updated && _current_goal_node != -1;
	res.exploration_finished = !_running;
	if (res.goal_available) {
//		ROS_INFO_STREAM(
//				"Current goal status " << (int)_prm.nodes[_current_goal_node].status);
		if (_rrg.nodes[_current_goal_node].status
				== rrg_nbv_exploration_msgs::Node::VISITED) {
//			ROS_INFO_STREAM(
//					"Current goal " << _current_goal_node << " from visited to active visited");
			_rrg.nodes[_current_goal_node].status =
					rrg_nbv_exploration_msgs::Node::ACTIVE_VISITED;
		} else if (_rrg.nodes[_current_goal_node].status
				== rrg_nbv_exploration_msgs::Node::INITIAL) {
//			ROS_INFO_STREAM(
//					"Current goal " << _current_goal_node << " from initial to active");
			_rrg.nodes[_current_goal_node].status =
					rrg_nbv_exploration_msgs::Node::ACTIVE;
		}
		res.goal = _rrg.nodes[_current_goal_node].position;
		res.best_yaw = _rrg.nodes[_current_goal_node].best_yaw;
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
	if (_current_goal_node != -1 && !_updating) {
		_updating = true;
		_rrg.nodes[_current_goal_node].status = req.status;
		updateCurrentGoal();
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

}
