#include "rrt_nbv_exploration/TreeConstructor.h"

namespace rrt_nbv_exploration {
TreeConstructor::TreeConstructor() {
}

TreeConstructor::~TreeConstructor() {
	//test for double free error
//	_tree_searcher.reset();
//	_gain_calculator.reset();
//	_collision_checker.reset();
//	_octree.reset();
//	_abstract_octree.reset();
}

void TreeConstructor::initialization(geometry_msgs::Point seed) {
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_max_range", _sensor_range, 5.0);
	_radius_search_range = pow(2 * _sensor_range, 2);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	private_nh.param("edge_length", _edge_length, 1.0);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("grid_map_resolution", _grid_map_resolution, 0.05);
	private_nh.param("exploration_finished_timer_duration",
			_exploration_finished_timer_duration, 10.0);
	private_nh.param("coupled_gain_calculation", _coupled_gain_calculation,
			false);
	private_nh.param("max_tree_nodes", _max_tree_nodes, 0);
	int rne_mode;
	private_nh.param("rne_mode", rne_mode, (int) RneMode::classic);
	_rne_mode = static_cast<RneMode>(rne_mode);

	ros::NodeHandle nh("rne");
	_rrt_publisher = nh.advertise<rrt_nbv_exploration_msgs::Tree>("rrt_tree",
			1);
	if (!_coupled_gain_calculation) {
		_node_to_update_publisher =
				nh.advertise<rrt_nbv_exploration_msgs::Node>("node_to_update",
						1);
		_updated_node_subscriber = nh.subscribe("updated_node", 1,
				&TreeConstructor::updatedNodeCallback, this);
	}
	_best_and_current_goal_publisher = nh.advertise<
			rrt_nbv_exploration_msgs::BestAndCurrentNode>("bestAndCurrentGoal",
			1);
	_request_goal_service = nh.advertiseService("requestGoal",
			&TreeConstructor::requestGoal, this);
	_request_path_service = nh.advertiseService("requestPath",
			&TreeConstructor::requestPath, this);
	_update_current_goal_service = nh.advertiseService("updateCurrentGoal",
			&TreeConstructor::updateCurrentGoal, this);

	_exploration_finished_timer = _nh.createTimer(
			ros::Duration(_exploration_finished_timer_duration),
			&TreeConstructor::explorationFinishedTimerCallback, this, false,
			false);

	_set_rrt_state_service = nh.advertiseService("setRrtState",
			&TreeConstructor::setRrtState, this);
	_get_rrt_state_service = nh.advertiseService("getRrtState",
			&TreeConstructor::getRrtState, this);
	_reset_rrt_state_service = nh.advertiseService("resetRrtState",
			&TreeConstructor::resetRrtState, this);

	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&TreeConstructor::convertOctomapMsgToOctree, this);

	_tree_searcher.reset(new TreeSearcher());
	_collision_checker.reset(new CollisionChecker());
	_tree_path_calculator.reset(new TreePathCalculator());
	_node_comparator.reset(new NodeComparator());
	if (_coupled_gain_calculation)
		_gain_calculator.reset(new GainCalculator());
	_running = false;
	initRrt(seed);
}

bool TreeConstructor::initRrt(const geometry_msgs::Point &seed) {
	_rrt.header.frame_id = "/map";
	_rrt.ns = "rrt_tree";
	_rrt.node_counter = 0;
	rrt_nbv_exploration_msgs::Node root;
	root.position = seed;
	root.position.z += _sensor_height;
	root.children_counter = 0;
	root.parent = -1;
	root.status = rrt_nbv_exploration_msgs::Node::VISITED;
	root.gain = -1;
	root.index = 0;
	root.distanceToParent = 0;
	root.pathToRobot.push_back(0);
	_rrt.nodes.push_back(root);
	_rrt.node_counter++;
	_rrt.root = 0;
	_rrt.nearest_node = 0;
	_current_goal_node = -1;
	_last_goal_node = 0;
	_last_updated_node = -1;
	_goal_updated = true;
	_updating = false;
	_sort_nodes_to_update = false;
	_last_robot_pos = seed;
	_tree_searcher->initialize(_rrt);
	if (_coupled_gain_calculation)
		_gain_calculator->precalculateGainPolls();
	else
		_nodes_to_update.push_back(0);
	_node_comparator->initialization();
	_generator.seed(time(NULL));
	return _collision_checker->initialize(seed);
}

void TreeConstructor::startRrtConstruction() {
	_rrt.nodes.clear();
	_node_comparator->clear();
	_nodes_to_update.clear();
	if (initRrt(_tree_path_calculator->getRobotPose().position)) {
		_running = true;
	} else {
		ROS_WARN_STREAM(
				"Unable to start RNE because of obstacles too close to the robot!");
	}
}

void TreeConstructor::stopRrtConstruction() {
	_running = false;
}

void TreeConstructor::runRrtConstruction() {
	_rrt.header.stamp = ros::Time::now();
	if (_running && _map_min_bounding[0] && _map_min_bounding[1]
			&& _map_min_bounding[2]) {
		determineNearestNodeToRobot();
		if (_max_tree_nodes <= 0
				|| _node_comparator->getListSize() <= _max_tree_nodes) {
			geometry_msgs::Point rand_sample;
			samplePoint(rand_sample);
			double min_distance;
			int nearest_node;
			_tree_searcher->findNearestNeighbour(rand_sample, min_distance,
					nearest_node);
			if (_edge_length > 0 ?
					min_distance >= pow(_edge_length, 2) :
					min_distance >= pow(_robot_radius, 2)) {
				placeNewNode(rand_sample, min_distance, nearest_node);
			}
		}
		_node_comparator->maintainList(_rrt);
		if (_current_goal_node == -1 && !_node_comparator->isEmpty()) {
			_current_goal_node = _node_comparator->getBestNode();
			_moved_to_current_goal = false;
			_goal_updated = true;
			_updating = false;
			ROS_INFO_STREAM("Current goal node set to " << _current_goal_node);
		}
		publishNodeWithBestGain();
		if (!_coupled_gain_calculation)
			publishNodeToUpdate();
		if (_nodes_to_update.empty() && _current_goal_node == -1) {
			_exploration_finished_timer.start();
		}
	}
	_rrt_publisher.publish(_rrt);
}

void TreeConstructor::samplePoint(geometry_msgs::Point &rand_sample) {
	std::uniform_real_distribution<double> x_distribution(_map_min_bounding[0],
			_map_max_bounding[0]);
	std::uniform_real_distribution<double> y_distribution(_map_min_bounding[1],
			_map_max_bounding[1]);
	rand_sample.x = x_distribution(_generator);
	rand_sample.y = y_distribution(_generator);
}

void TreeConstructor::alignPointToGridMap(geometry_msgs::Point &rand_sample) {
	rand_sample.x = (round(rand_sample.x / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
	rand_sample.y = (round(rand_sample.y / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
}

void TreeConstructor::placeNewNode(geometry_msgs::Point rand_sample,
		double min_distance, int nearest_node) {
	double distance = sqrt(min_distance);
	if (_edge_length > 0) {
		double x = _rrt.nodes[nearest_node].position.x
				- (_edge_length
						* (_rrt.nodes[nearest_node].position.x - rand_sample.x)
						/ distance);
		double y = _rrt.nodes[nearest_node].position.y
				- (_edge_length
						* (_rrt.nodes[nearest_node].position.y - rand_sample.y)
						/ distance);
		rand_sample.x = x;
		rand_sample.y = y;
		rand_sample.z = _rrt.nodes[nearest_node].position.z;
	}
	alignPointToGridMap(rand_sample);
	rrt_nbv_exploration_msgs::Node node;
	if (_collision_checker->steer(node, _rrt.nodes[nearest_node], rand_sample,
			_edge_length > 0 ? _edge_length : distance)) {
		node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
		node.gain = -1;
		node.parent = nearest_node;
		node.distanceToParent = _edge_length > 0 ? _edge_length : distance;
		node.index = _rrt.node_counter;
		_tree_path_calculator->initializePathToRobot(node, _rrt.node_counter,
				_rrt.nodes[nearest_node].pathToRobot,
				_rrt.nodes[nearest_node].distanceToRobot);
		if (_coupled_gain_calculation) {
			_gain_calculator->calculateGain(node);
			if (node.status != rrt_nbv_exploration_msgs::Node::EXPLORED)
				_node_comparator->addNode(node.index);
		} else {
			_nodes_to_update.push_back(_rrt.node_counter);
			_sort_nodes_to_update = true;
		}
		_rrt.nodes.push_back(node);
		_rrt.nodes[nearest_node].children.push_back(_rrt.node_counter);
		_rrt.nodes[nearest_node].children_counter++;
		_rrt.node_counter++;
		_tree_searcher->rebuildIndex(_rrt);
		_exploration_finished_timer.stop();
	}
}

void TreeConstructor::determineNearestNodeToRobot() {
	geometry_msgs::Point pos = _tree_path_calculator->getRobotPose().position;
	if (pos.x != _last_robot_pos.x || pos.y != _last_robot_pos.y
			|| pos.z != _last_robot_pos.z) {
		_last_robot_pos = pos;
		double min_distance;
		int nearest_node;
		_tree_searcher->findNearestNeighbour(pos, min_distance, nearest_node);
		if (nearest_node != _rrt.nearest_node) {
			_moved_to_current_goal = true;
			if (_tree_path_calculator->neighbourNodes(_rrt, _rrt.nearest_node,
					nearest_node)) {
				_tree_path_calculator->updatePathsToRobot(_rrt.nearest_node,
						nearest_node, _rrt);
			} else {
				_tree_path_calculator->recalculatePathsToRobot(
						_rrt.nearest_node, nearest_node, _rrt);
			}
			_node_comparator->robotMoved();
			_rrt.nearest_node = nearest_node;
		}
	}
}

void TreeConstructor::publishNodeWithBestGain() {
	rrt_nbv_exploration_msgs::BestAndCurrentNode msg;
	msg.current_goal = _current_goal_node;
	msg.best_node =
			_node_comparator->isEmpty() ?
					_current_goal_node : _node_comparator->getBestNode();
	msg.goal_updated = _goal_updated;
	_best_and_current_goal_publisher.publish(msg);
}

void TreeConstructor::updateNodes(geometry_msgs::Point center_node) {
//ROS_INFO("start updating nodes");
	std::vector<int> updatable_nodes = _tree_searcher->searchInRadius(
			center_node, _radius_search_range);
	for (auto iterator : updatable_nodes) {
		if (_rrt.nodes[iterator].status
				!= rrt_nbv_exploration_msgs::Node::EXPLORED
				&& _rrt.nodes[iterator].status
						!= rrt_nbv_exploration_msgs::Node::FAILED) {
			if (_coupled_gain_calculation) {
				_gain_calculator->calculateGain(_rrt.nodes[iterator]);
				if (_rrt.nodes[iterator].status
						== rrt_nbv_exploration_msgs::Node::EXPLORED
						|| _rrt.nodes[iterator].status
								== rrt_nbv_exploration_msgs::Node::FAILED)
					_node_comparator->removeNode(iterator);
				else
					_node_comparator->setSortList();
			} else {
				_node_comparator->removeNode(iterator);
				_nodes_to_update.push_back(iterator);
				_sort_nodes_to_update = true;
				_rrt.nodes[iterator].gain = -1;
			}
		}
	}
}

void TreeConstructor::sortNodesToUpdateByDistanceToRobot() {
	_nodes_to_update.sort([this](int node_one, int node_two) {
		return compareNodeDistancesToRobot(node_one, node_two);
	});
	_sort_nodes_to_update = false;

}

bool TreeConstructor::compareNodeDistancesToRobot(const int &node_one,
		const int &node_two) {
	return _rrt.nodes[node_one].distanceToRobot
			<= _rrt.nodes[node_two].distanceToRobot;
}

void TreeConstructor::publishNodeToUpdate() {
	if (!_nodes_to_update.empty()) {
		if (_sort_nodes_to_update)
			sortNodesToUpdateByDistanceToRobot();
		_node_to_update_publisher.publish(_rrt.nodes[_nodes_to_update.front()]);
	}
}

void TreeConstructor::updateCurrentGoal() {
//	ROS_INFO_STREAM(
//			"Update current goal " << _current_goal_node << " with status " << (int)_rrt.nodes[_current_goal_node].status);
	if (_rne_mode == RneMode::receding
			|| _rne_mode == RneMode::receding_horizon) {
		if (_rrt.nodes[_current_goal_node].status
				!= rrt_nbv_exploration_msgs::Node::ABORTED
				|| (_rrt.nodes[_current_goal_node].status
						== rrt_nbv_exploration_msgs::Node::ABORTED
						&& _moved_to_current_goal)) { //only restart if robot moved towards goal when aborted
			startRrtConstruction();
			return;
		}
	}
	geometry_msgs::Point update_center;
	switch (_rrt.nodes[_current_goal_node].status) {
	case rrt_nbv_exploration_msgs::Node::EXPLORED:
		ROS_INFO("goal explored");
		_node_comparator->removeNode(_current_goal_node);
		update_center = _rrt.nodes[_current_goal_node].position;
		updateNodes(update_center);
		break;
	case rrt_nbv_exploration_msgs::Node::VISITED:
		if (!_coupled_gain_calculation) {
			_node_comparator->removeNode(_current_goal_node);
			_nodes_to_update.push_front(_current_goal_node);
			_sort_nodes_to_update = true;
		}
		_rrt.nodes[_current_goal_node].gain = 0;
		update_center = _rrt.nodes[_current_goal_node].position;
		updateNodes(update_center);
		break;
	case rrt_nbv_exploration_msgs::Node::ABORTED:
		ROS_INFO("goal aborted");
		_rrt.nodes[_current_goal_node].status =
				rrt_nbv_exploration_msgs::Node::INITIAL;
		if (_moved_to_current_goal) {
			ROS_INFO("update nodes");
			if (!_coupled_gain_calculation) {
				_node_comparator->removeNode(_current_goal_node);
				_nodes_to_update.push_back(_current_goal_node);
				_sort_nodes_to_update = true;
			}
			update_center = _last_robot_pos;
			updateNodes(update_center);
		}
		break;
	case rrt_nbv_exploration_msgs::Node::FAILED:
		ROS_INFO("goal failed");
		_node_comparator->removeNode(_current_goal_node);
		_rrt.nodes[_current_goal_node].gain = 0;
		if (_moved_to_current_goal) {
			update_center = _last_robot_pos;
			updateNodes(update_center);
		}
		break;
	default:
		//active or waiting (should not occur)
		ROS_INFO("goal active or waiting");
		return;
	}
	_last_goal_node = (_current_goal_node == -1 ? 0 : _current_goal_node);
	_current_goal_node = -1;
}

void TreeConstructor::updatedNodeCallback(
		const rrt_nbv_exploration_msgs::Node::ConstPtr &updated_node) {
	if (updated_node->index != _last_updated_node
			|| _rrt.nodes[_last_updated_node].gain != updated_node->gain
			|| _rrt.nodes[_last_updated_node].best_yaw != updated_node->best_yaw
			|| _rrt.nodes[_last_updated_node].status != updated_node->status) {
		_rrt.nodes[updated_node->index].gain = updated_node->gain;
		_rrt.nodes[updated_node->index].best_yaw = updated_node->best_yaw;
		_rrt.nodes[updated_node->index].status = updated_node->status;
		_rrt.nodes[updated_node->index].position.z = updated_node->position.z;
		_last_updated_node = updated_node->index;
		_nodes_to_update.remove(updated_node->index);
		if (updated_node->status != rrt_nbv_exploration_msgs::Node::EXPLORED
				&& updated_node->status
						!= rrt_nbv_exploration_msgs::Node::FAILED) {
			_node_comparator->addNode(updated_node->index);
		}
		publishNodeToUpdate(); //if gain calculation is faster than update frequency, this needs to be called
	}
}

void TreeConstructor::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr &map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
	updateMapDimensions();
}

void TreeConstructor::updateMapDimensions() {
	_octree->getMetricMin(_map_min_bounding[0], _map_min_bounding[1],
			_map_min_bounding[2]);
	_octree->getMetricMax(_map_max_bounding[0], _map_max_bounding[1],
			_map_max_bounding[2]);
}

void TreeConstructor::explorationFinishedTimerCallback(
		const ros::TimerEvent &event) {
	ROS_INFO_STREAM("Exploration finished");
	_running = false;
	_exploration_finished_timer.stop();
}

bool TreeConstructor::requestGoal(
		rrt_nbv_exploration_msgs::RequestGoal::Request &req,
		rrt_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.goal_available = _goal_updated && _current_goal_node != -1;
	res.exploration_finished = !_running;
	if (res.goal_available) {
//		ROS_INFO_STREAM(
//				"Current goal status " << (int)_rrt.nodes[_current_goal_node].status);
		if (_rrt.nodes[_current_goal_node].status
				== rrt_nbv_exploration_msgs::Node::VISITED) {
//			ROS_INFO_STREAM(
//					"Current goal " << _current_goal_node << " from visited to active visited");
			_rrt.nodes[_current_goal_node].status =
					rrt_nbv_exploration_msgs::Node::ACTIVE_VISITED;
		} else if (_rrt.nodes[_current_goal_node].status
				== rrt_nbv_exploration_msgs::Node::INITIAL) {
//			ROS_INFO_STREAM(
//					"Current goal " << _current_goal_node << " from initial to active");
			_rrt.nodes[_current_goal_node].status =
					rrt_nbv_exploration_msgs::Node::ACTIVE;
		}
		res.goal = _rrt.nodes[_current_goal_node].position;
		res.best_yaw = _rrt.nodes[_current_goal_node].best_yaw;
		_goal_updated = false;
	}
	return true;
}

bool TreeConstructor::requestPath(
		rrt_nbv_exploration_msgs::RequestPath::Request &req,
		rrt_nbv_exploration_msgs::RequestPath::Response &res) {
	if (_current_goal_node != -1) {
		std::vector<geometry_msgs::PoseStamped> rrt_path;
		_tree_path_calculator->getNavigationPath(rrt_path, _rrt,
				_current_goal_node);
		res.path = rrt_path;
		return true;
	}
	return false;
}

bool TreeConstructor::updateCurrentGoal(
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Response &res) {
	if (_current_goal_node != -1 && !_updating) {
		_updating = true;
		_rrt.nodes[_current_goal_node].status = req.status;
		updateCurrentGoal();
	}
	res.success = true;
	res.message = "Changed current goal's status";
	return true;
}

bool TreeConstructor::setRrtState(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (req.data) {
		if (_running) {
			res.success = false;
			res.message = "Tree construction already running";
		} else {
			startRrtConstruction();
			res.success = true;
			res.message = "Tree construction started";
		}
	} else {
		if (!_running) {
			res.success = false;
			res.message = "Tree construction already stopped";
		} else {
			stopRrtConstruction();
			res.success = true;
			res.message = "Tree construction stopped";
		}
	}
	return true;
}

bool TreeConstructor::getRrtState(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	res.success = _running;
	res.message = "Tree construction status sent";
	return true;
}

bool TreeConstructor::resetRrtState(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	_running = false;
	_rrt.nodes.clear();
	_node_comparator->clear();
	_nodes_to_update.clear();
	initRrt(geometry_msgs::Point());
	res.message = true;
	res.message = "Tree reset";
	return true;
}

}
