#include "rrt_nbv_exploration/TreeConstructor.h"

namespace rrt_nbv_exploration {
TreeConstructor::TreeConstructor() :
		_map_dimensions { 0.0, 0.0, 0.0 }, _nodes_ordered_by_gain(
				[this](int node1, int node2) {return _rrt.nodes[node1].gain < _rrt.nodes[node2].gain;}) {
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
	private_nh.param("sensor_range", _sensor_range, 5.0);
	_radius_search_range = pow(2 * _sensor_range, 2);
	private_nh.param("sensor_height", _sensor_height, 0.5);

	ros::NodeHandle nh("rne");
	_rrt_publisher = nh.advertise<rrt_nbv_exploration_msgs::Tree>("rrt_tree",
			1);
	_best_and_current_goal_publisher = nh.advertise<
			rrt_nbv_exploration_msgs::BestAndCurrentNode>("bestAndCurrentGoal",
			1);
	_request_goal_service = nh.advertiseService("requestGoal",
			&TreeConstructor::requestGoal, this);
	_update_current_goal_service = nh.advertiseService("updateCurrentGoal",
			&TreeConstructor::updateCurrentGoal, this);

	_set_rrt_state_service = nh.advertiseService("setRrtState",
			&TreeConstructor::setRrtState, this);
	_get_rrt_state_service = nh.advertiseService("getRrtState",
			&TreeConstructor::getRrtState, this);
	_reset_rrt_state_service = nh.advertiseService("resetRrtState",
			&TreeConstructor::resetRrtState, this);

	_octomap_sub = _nh.subscribe("octomap_binary", 1,
			&TreeConstructor::convertOctomapMsgToCctree, this);

	_tree_searcher.reset(new TreeSearcher());
	_gain_calculator.reset(new GainCalculator());
	_collision_checker.reset(new CollisionChecker());

	_running = false;
	initRrt(seed);
}

void TreeConstructor::initRrt(const geometry_msgs::Point& seed) {
	_rrt.header.frame_id = "/map";
	_rrt.ns = "rrt_tree";
	_rrt.node_counter = 0;
	rrt_nbv_exploration_msgs::Node root;
	root.position = seed;
	root.position.z = _sensor_height;
	root.children_counter = 0;
	root.status = rrt_nbv_exploration_msgs::Node::VISITED;
	_rrt.nodes.push_back(root);
	_rrt.node_counter++;
	_rrt.root = 0;
	_current_goal_node = -1;
	_last_goal_node = 0;
	//_nodes_ordered_by_gain.insert(_current_goal_node);
	_tree_searcher->initialize(_rrt);
	_generator.seed(time(NULL));
}

void TreeConstructor::startRrtConstruction() {
	_running = true;
}

void TreeConstructor::stopRrtConstruction() {
	_running = false;
}

void TreeConstructor::runRrtConstruction() {
	//ROS_INFO_STREAM("Constructing");
	_rrt.header.stamp = ros::Time::now();
	if (_running && _map_dimensions[0] && _map_dimensions[1]
			&& _map_dimensions[2]) {
		geometry_msgs::Point rand_sample;
		if (samplePoint(rand_sample)) {
			double min_distance;
			int nearest_node;
			_tree_searcher->findNearestNeighbour(rand_sample, min_distance,
					nearest_node);
			placeNewNode(rand_sample, min_distance, nearest_node);
		}
		if (_current_goal_node == -1 && !_nodes_ordered_by_gain.empty()) {
			_current_goal_node = *_nodes_ordered_by_gain.rbegin();
			ROS_INFO_STREAM("Current goal node set to " << _current_goal_node);
		}
		publishNodeWithBestGain();
	}
	_rrt_publisher.publish(_rrt);
}

bool TreeConstructor::samplePoint(geometry_msgs::Point& rand_sample) {
	std::uniform_real_distribution<double> x_distribution(
			-_map_dimensions[0] / 2, _map_dimensions[0] / 2);
	std::uniform_real_distribution<double> y_distribution(
			-_map_dimensions[1] / 2, _map_dimensions[1] / 2);
	rand_sample.x = x_distribution(_generator);
	rand_sample.y = y_distribution(_generator);
	rand_sample.z = 0.2;
	octomap::point3d rand_point(rand_sample.x, rand_sample.y, rand_sample.z);
	octomap::OcTreeNode* octree_node = _octree->search(rand_point);
	if (octree_node != NULL && !_octree->isNodeOccupied(octree_node)) {
		return true;
	}
	return false;
}

void TreeConstructor::placeNewNode(geometry_msgs::Point rand_sample,
		double min_distance, int nearest_node) {
	rrt_nbv_exploration_msgs::Node node;
	if (_collision_checker->steer(node, _rrt.nodes[nearest_node], rand_sample,
			min_distance, _octree)) {
		_gain_calculator->calculateGain(node, _octree);
		_rrt.nodes.push_back(node);
		_nodes_ordered_by_gain.insert(_rrt.node_counter);
		_rrt.nodes[nearest_node].children.push_back(_rrt.node_counter);
		_rrt.nodes[nearest_node].children_counter++;
		_rrt.node_counter++;
		_tree_searcher->rebuildIndex(_rrt);
	}
}

void TreeConstructor::publishNodeWithBestGain() {
	rrt_nbv_exploration_msgs::BestAndCurrentNode msg;
	msg.current_goal = _current_goal_node;
	msg.best_node =
			_nodes_ordered_by_gain.empty() ?
					_current_goal_node : *_nodes_ordered_by_gain.rbegin();
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
			if (_nodes_ordered_by_gain.find(iterator)
					!= _nodes_ordered_by_gain.end()) {
				_nodes_ordered_by_gain.erase(iterator);
			}
			_gain_calculator->calculateGain(_rrt.nodes[iterator], _octree);
			if (_rrt.nodes[iterator].status
					!= rrt_nbv_exploration_msgs::Node::EXPLORED) {
				_nodes_ordered_by_gain.insert(iterator);
			}
		}
	}
	for (auto it : _nodes_ordered_by_gain) {
		ROS_INFO_STREAM(
				"Node " << it << " with gain " << _rrt.nodes[it].gain << " and status " << _rrt.nodes[it].status);
	}
}

void TreeConstructor::updateCurrentGoal() {
	ROS_INFO_STREAM("Update current goal " << _current_goal_node);
	geometry_msgs::Point update_center;
	switch (_rrt.nodes[_current_goal_node].status) {
	case rrt_nbv_exploration_msgs::Node::EXPLORED:
		ROS_INFO("goal explored");
		_nodes_ordered_by_gain.erase(_current_goal_node);
		update_center = _rrt.nodes[_current_goal_node].position;
		break;
	case rrt_nbv_exploration_msgs::Node::VISITED:
		ROS_INFO("goal visited");
		update_center = _rrt.nodes[_current_goal_node].position;
		break;
	case rrt_nbv_exploration_msgs::Node::ABORTED:
		ROS_INFO("goal aborted");
		_rrt.nodes[_current_goal_node].status =
				rrt_nbv_exploration_msgs::Node::VISITED;
		update_center = _collision_checker->getRobotPose().position;
		break;
	case rrt_nbv_exploration_msgs::Node::FAILED:
		ROS_INFO("goal failed");
		_nodes_ordered_by_gain.erase(_current_goal_node);
		update_center = _collision_checker->getRobotPose().position;
		break;
	default:    //active or waiting
		break;
	}
	updateNodes(update_center);
	_current_goal_node = -1;

	//ROS_INFO("Set current goal to %i", _current_goal_node);
}

void TreeConstructor::convertOctomapMsgToCctree(
		const octomap_msgs::Octomap::ConstPtr& map_msg) {
	//ROS_INFO_STREAM("Receive octomap");
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	//ROS_INFO_STREAM("first abstract octomap");
	_octree = boost::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
	//ROS_INFO_STREAM("dynamic cast octomap");
	updateMapDimensions();
}

void TreeConstructor::updateMapDimensions() {
	//ROS_INFO_STREAM("Octomap dimensions");
	_octree->getMetricSize(_map_dimensions[0], _map_dimensions[1],
			_map_dimensions[2]);
}

bool TreeConstructor::requestGoal(
		rrt_nbv_exploration_msgs::RequestGoal::Request &req,
		rrt_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.goal_available = _current_goal_node != -1;
	if (res.goal_available) {
		if (_rrt.nodes[_current_goal_node].status
				== rrt_nbv_exploration_msgs::Node::VISITED) {
			_rrt.nodes[_current_goal_node].status =
					rrt_nbv_exploration_msgs::Node::ACTIVE_VISITED;
		} else {
			_rrt.nodes[_current_goal_node].status =
					rrt_nbv_exploration_msgs::Node::ACTIVE;
		}
		res.goal = _rrt.nodes[_current_goal_node].position;
		res.best_yaw = _rrt.nodes[_current_goal_node].best_yaw;
	}
	return true;
}

bool TreeConstructor::updateCurrentGoal(
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Response &res) {
	_rrt.nodes[_current_goal_node].status = req.status;
	updateCurrentGoal();
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
	_nodes_ordered_by_gain.clear();
	initRrt(geometry_msgs::Point());
	res.message = true;
	res.message = "Tree reset";
	return true;
}

}
