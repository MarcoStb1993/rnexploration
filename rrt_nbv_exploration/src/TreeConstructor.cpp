#include "rrt_nbv_exploration/TreeConstructor.h"

namespace rrt_nbv_exploration {
TreeConstructor::TreeConstructor() {
}

TreeConstructor::~TreeConstructor() {
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
	private_nh.param("n_max", _n_max, 15);
	private_nh.param("n_tol", _n_tol, 50);
	private_nh.param("max_consecutive_failed_goals",
			_max_consecutive_failed_goals, 5);

	ros::NodeHandle nh("rne");
	_rrt_publisher = nh.advertise<rrt_nbv_exploration_msgs::Tree>("rrt_tree",
			1);
	_request_goal_service = nh.advertiseService("requestGoal",
			&TreeConstructor::requestGoal, this);
	_request_path_service = nh.advertiseService("requestPath",
			&TreeConstructor::requestPath, this);
	_update_current_goal_service = nh.advertiseService("updateCurrentGoal",
			&TreeConstructor::updateCurrentGoal, this);

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
	_gain_calculator.reset(new GainCalculator());
	_running = false;
	_construction_running = false;
	_consecutive_failed_goals = 0;
	_gain_calculator->precalculateGainPolls();
	initRrt(seed);
}

bool TreeConstructor::initRrt(const geometry_msgs::Point &seed) {
	_rrt.header.frame_id = "/map";
	_rrt.ns = "rrt_tree";
	_rrt.node_counter = 0;
	if (_best_branch.node_counter > 0) {
		_rrt.nodes = _best_branch.nodes;
		_rrt.node_counter = _best_branch.node_counter;
		resetBestBranch();
	} else {
		rrt_nbv_exploration_msgs::Node root;
		root.position = seed;
		root.position.z += _sensor_height;
		root.children_counter = 0;
		root.parent = -1;
		root.status = rrt_nbv_exploration_msgs::Node::VISITED;
		root.index = 0;
		root.distanceToParent = 0;
		root.pathToRobot.push_back(0);
		_rrt.nodes.push_back(root);
		_rrt.node_counter++;
	}
	_rrt.root = 0;
	_rrt.nearest_node = 0;
	_current_goal_node = -1;
	_updating = false;
	_goal_updated = false;
	_tree_searcher->initialize(_rrt);
	_node_comparator->initialization();
	_generator.seed(time(NULL));
	return _collision_checker->initialize(seed);
}

void TreeConstructor::storeBestBranch(std::vector<int> nodes) {
	if (nodes.size() > 0) {
		_best_branch.node_counter = nodes.size();
		rrt_nbv_exploration_msgs::Node root;
		root.position = _rrt.nodes[nodes[0]].position;
		root.parent = -1;
		root.status =
				_rrt.nodes[nodes[0]].status
						== rrt_nbv_exploration_msgs::Node::ACTIVE_VISITED ?
						rrt_nbv_exploration_msgs::Node::VISITED :
						_rrt.nodes[nodes[0]].status;
		root.gain = _rrt.nodes[nodes[0]].gain;
		root.best_yaw = _rrt.nodes[nodes[0]].best_yaw;
		root.index = 0;
		root.distanceToParent = 0;
		root.distanceToRobot = 0;
		root.pathToRobot.push_back(0);
		if (nodes.size() > 1) {
			root.children.push_back(1);
			root.children_counter = 1;
		}
		_best_branch.nodes.push_back(root);
		for (int i = 1; i < nodes.size(); i++) {
			rrt_nbv_exploration_msgs::Node node;
			node.position = _rrt.nodes[nodes[i]].position;
			node.parent = i - 1;
			node.status = _rrt.nodes[nodes[i]].status;
			node.gain = _rrt.nodes[nodes[i]].gain;
			node.best_yaw = _rrt.nodes[nodes[i]].best_yaw;
			_gain_calculator->calculateGain(node);
			node.index = i;
			node.distanceToParent = _rrt.nodes[nodes[i]].distanceToParent;
			node.distanceToRobot = sqrt(
					pow(root.position.x - node.position.x, 2)
							+ pow(root.position.y - node.position.y, 2));
			for (int p = 0; p <= i; p++) {
				node.pathToRobot.push_back(p);
			}
			if (i + 1 < nodes.size()) {
				node.children.push_back(i + 1);
				node.children_counter = 1;
			}
			_best_branch.nodes.push_back(node);
		}
	} else {
		resetBestBranch();
	}
}

void TreeConstructor::resetBestBranch() {
	_best_branch.nodes.clear();
	_best_branch.node_counter = 0;
}

void TreeConstructor::startRrtConstruction() {
	_rrt.nodes.clear();
	_node_comparator->clear();
	if (initRrt(_tree_path_calculator->getRobotPose().position)) {
		_running = true;
		_construction_running = true;
		_gain_calculator->calculateGain(_rrt.nodes[0]);
		for (auto node : _rrt.nodes) {
			if (node.gain > 0)
				_node_comparator->addNode(node.index);
		}
		runRrtConstruction();

	} else {
		ROS_WARN_STREAM(
				"Unable to start RNE because of obstacles too close to the robot!");
	}
}

void TreeConstructor::stopRrtConstruction() {
	_running = false;
	_construction_running = false;
	_consecutive_failed_goals = 0;
}

void TreeConstructor::publishRrt() {
	_rrt.header.stamp = ros::Time::now();
	_rrt_publisher.publish(_rrt);
}

void TreeConstructor::runRrtConstruction() {
	_rrt_start_time = ros::Time::now();
	while (_running && _construction_running && _map_min_bounding[0]
			&& _map_min_bounding[1] && _map_min_bounding[2]) {
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
		//only set goal when n_max reached and only add goals with gain>0,
		if (_rrt.node_counter >= _n_max && !_node_comparator->isEmpty()) {
			_node_comparator->maintainList(_rrt);
			_current_goal_node = _node_comparator->getBestNode();
			_construction_running = false;
			_updating = false;
			_goal_updated = true;
			ROS_INFO_STREAM("Current goal node set to " << _current_goal_node);
		} else if (_rrt.node_counter >= _n_tol) {
			ROS_INFO_STREAM("Exploration finished");
			_running = false;
			_construction_running = false;
		} else if (ros::Time::now().toSec() - _rrt_start_time.toSec()
				> _exploration_finished_timer_duration) {
			if (!_node_comparator->isEmpty()) {
				_node_comparator->maintainList(_rrt);
				_current_goal_node = _node_comparator->getBestNode();
				_construction_running = false;
				_updating = false;
				_goal_updated = true;
				ROS_INFO_STREAM(
						"Timer set current goal node to " << _current_goal_node);
			} else {
				ROS_INFO_STREAM("Exploration finished");
				_running = false;
				_construction_running = false;
			}
		}
	}
}

void TreeConstructor::samplePoint(geometry_msgs::Point &rand_sample) {
	std::uniform_real_distribution<double> x_distribution(_map_min_bounding[0],
			_map_max_bounding[0]);
	std::uniform_real_distribution<double> y_distribution(_map_min_bounding[1],
			_map_max_bounding[1]);
	rand_sample.x = x_distribution(_generator);
	rand_sample.y = y_distribution(_generator);
}

void TreeConstructor::alignPointToGridMap(geometry_msgs::Point &rand_sample,
		int nearest_node, double &distance) {
	rand_sample.x = (round(rand_sample.x / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
	rand_sample.y = (round(rand_sample.y / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
	distance = sqrt(
			pow(rand_sample.x - _rrt.nodes[nearest_node].position.x, 2)
					+ pow(rand_sample.y - _rrt.nodes[nearest_node].position.y,
							2));
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
	alignPointToGridMap(rand_sample, nearest_node, distance);
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
		_gain_calculator->calculateGain(node);
		_rrt.nodes.push_back(node);
		_rrt.nodes[nearest_node].children.push_back(_rrt.node_counter);
		_rrt.nodes[nearest_node].children_counter++;
		_rrt.node_counter++;
		_tree_searcher->rebuildIndex(_rrt);
		if (node.status != rrt_nbv_exploration_msgs::Node::EXPLORED)
			_node_comparator->addNode(node.index);
		publishRrt();
		_rrt_start_time = ros::Time::now();
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

bool TreeConstructor::requestGoal(
		rrt_nbv_exploration_msgs::RequestGoal::Request &req,
		rrt_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.goal_available = _goal_updated && _current_goal_node != -1;
	res.exploration_finished = !_running;
	if (res.goal_available) {
		if (_rrt.nodes[_current_goal_node].status
				== rrt_nbv_exploration_msgs::Node::VISITED) {
			_rrt.nodes[_current_goal_node].status =
					rrt_nbv_exploration_msgs::Node::ACTIVE_VISITED;
		} else if (_rrt.nodes[_current_goal_node].status
				== rrt_nbv_exploration_msgs::Node::INITIAL) {
			_rrt.nodes[_current_goal_node].status =
					rrt_nbv_exploration_msgs::Node::ACTIVE;
		}
		res.goal = _rrt.nodes[_current_goal_node].position;
		// check if goal has gain>0 or is only explored node on best branch
		if (_rrt.nodes[_current_goal_node].gain > 0
				|| _rrt.nodes[_current_goal_node].children_counter == 0) {
			res.best_yaw = _rrt.nodes[_current_goal_node].best_yaw;
		} else {
			// get orientation towards next node in best branch (guess based on child with most gain, default: first child)
			int best_child = 0;
			double best_gain = -1;
			for (auto child : _rrt.nodes[_current_goal_node].children) {
				if (_rrt.nodes[child].gain > best_gain) {
					best_child = child;
					best_gain = _rrt.nodes[child].gain;
				}
			}
			double yaw = atan2(
					_rrt.nodes[best_child].position.y
							- _rrt.nodes[_current_goal_node].position.y,
					_rrt.nodes[best_child].position.x
							- _rrt.nodes[_current_goal_node].position.x);
			res.best_yaw = yaw * 180 / M_PI;
			_rrt.nodes[_current_goal_node].best_yaw = res.best_yaw;
		}
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
		if (req.status == rrt_nbv_exploration_msgs::Node::VISITED
				|| req.status == rrt_nbv_exploration_msgs::Node::EXPLORED) {
			storeBestBranch(_node_comparator->getBestBranch());
			_consecutive_failed_goals = 0;
		} else if (req.status == rrt_nbv_exploration_msgs::Node::FAILED) {
			if (++_consecutive_failed_goals >= _max_consecutive_failed_goals) {
				ROS_INFO_STREAM("Exploration aborted, robot stuck");
				_rrt.node_counter = -1;
				stopRrtConstruction();
				return true;
			}
			resetBestBranch();
		}
		_updating = true;
		startRrtConstruction();
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
	initRrt(geometry_msgs::Point());
	res.message = true;
	res.message = "Tree reset";
	return true;
}

}
