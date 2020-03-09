#include "rrt_nbv_exploration/TreeConstructor.h"

namespace rrt_nbv_exploration {
TreeConstructor::TreeConstructor() :
		_map_dimensions { 0.0, 0.0, 0.0 }, _nodes_ordered_by_gain(
				[this](int node1, int node2) {return _rrt.nodes[node1].gain < _rrt.nodes[node2].gain;}) {
}

TreeConstructor::~TreeConstructor() {
	//ROS_INFO_STREAM("Alive");
	_tree_searcher.reset();
	_gain_calculator.reset();
	_collision_checker.reset();
	_abstract_octree.reset();
	_octree.reset();
}

void TreeConstructor::initialization(geometry_msgs::Point seed) {
	//ROS_INFO_STREAM("Init start");
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_range", _sensor_range, 5.0);
	_radius_search_range = pow(2 * _sensor_range, 2);

	ros::NodeHandle nh("rne");
	_rrt_publisher = nh.advertise<rrt_nbv_exploration_msgs::Tree>("rrt_tree", 1);
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
			&TreeConstructor::convert_octomap_msg_to_octree, this);

	_tree_searcher.reset(new TreeSearcher());
	_gain_calculator.reset(new GainCalculator());
	_collision_checker.reset(new CollisionChecker());

	_running = false;
	initRrt(seed);
	//ROS_INFO_STREAM("Init finished");
}

void TreeConstructor::initRrt(const geometry_msgs::Point& seed) {
	_rrt.header.frame_id = "/map";
	_rrt.ns = "rrt_tree";
	_rrt.node_counter = 0;
	rrt_nbv_exploration_msgs::Node root;
	root.position = seed;
	root.children_counter = 0;
	root.status = rrt_nbv_exploration_msgs::Node::EXPLORED;
	_rrt.nodes.push_back(root);
	_rrt.node_counter++;
	_rrt.root = 0;
	_current_goal_node = 0;
	_last_goal_node = 0;
	_nodes_ordered_by_gain.insert(_current_goal_node);
	_tree_searcher->initialize(_rrt);
	_generator.seed(time(NULL));
}

void TreeConstructor::start_rrt_construction() {
	_running = true;
}

void TreeConstructor::stop_rrt_construction() {
	_running = false;
}

void TreeConstructor::run_rrt_construction() {
	//ROS_INFO_STREAM("Constructing");
	_rrt.header.stamp = ros::Time::now();
	if (_running && _map_dimensions[0] && _map_dimensions[1]
			&& _map_dimensions[2]) {
		geometry_msgs::Point rand_sample;
		if (sample_point(rand_sample)) {
			double min_distance;
			int nearest_node;
			_tree_searcher->find_nearest_neighbour(rand_sample, min_distance,
					nearest_node);
			place_new_node(rand_sample, min_distance, nearest_node);
		}
		publish_node_with_best_gain();
		//update_current_goal();
	}
	_rrt_publisher.publish(_rrt);
}

bool TreeConstructor::sample_point(geometry_msgs::Point& rand_sample) {
	//ROS_INFO_STREAM("Sample point");
	//randomly sampled point
	std::uniform_real_distribution<double> x_distribution(
			-_map_dimensions[0] / 2, _map_dimensions[0] / 2);
	std::uniform_real_distribution<double> y_distribution(
			-_map_dimensions[1] / 2, _map_dimensions[1] / 2);
	std::uniform_real_distribution<double> z_distribution(0,
			_map_dimensions[2]);
	rand_sample.x = x_distribution(_generator);
	rand_sample.y = y_distribution(_generator);
	rand_sample.z = z_distribution(_generator);
	octomap::point3d rand_point(rand_sample.x, rand_sample.y, rand_sample.z);
	octomap::OcTreeNode* octree_node = _octree->search(rand_point);
	if (octree_node != NULL && !_octree->isNodeOccupied(octree_node)) {
		//ROS_INFO_STREAM("Sample point success");
		return true;
	}
	return false;
}

void TreeConstructor::place_new_node(geometry_msgs::Point rand_sample,
		double min_distance, int nearest_node) {
	//ROS_INFO_STREAM("Place new node");
	//distance metric/kinematic constraints
	rrt_nbv_exploration_msgs::Node node;
	if (_collision_checker->steer(node, _rrt.nodes[nearest_node], rand_sample,
			min_distance, _octree)) {
		_gain_calculator->calculate_gain(node, _octree);
		_rrt.nodes.push_back(node);
		_nodes_ordered_by_gain.insert(_rrt.node_counter);
		_rrt.nodes[nearest_node].children.push_back(_rrt.node_counter);
		_rrt.nodes[nearest_node].children_counter++;
		_rrt.node_counter++;
		_tree_searcher->rebuildIndex(_rrt);
		//ROS_INFO_STREAM("Placed it");
	}
}

void TreeConstructor::publish_node_with_best_gain() {
	rrt_nbv_exploration_msgs::BestAndCurrentNode msg;
	msg.best_node = *_nodes_ordered_by_gain.rbegin();
	msg.current_goal = _current_goal_node;
	_best_and_current_goal_publisher.publish(msg);
}

void TreeConstructor::update_nodes(
		rrt_nbv_exploration_msgs::Node& center_node) {
	//ROS_INFO("start updating nodes");
	std::vector<int> updatable_nodes = _tree_searcher->search_in_radius(
			center_node.position, _radius_search_range);
	for (auto iterator : updatable_nodes) {
		//ROS_INFO("update node %i", iterator);
		_gain_calculator->calculate_gain(_rrt.nodes[iterator], _octree);
	}
}

void TreeConstructor::update_current_goal() {
	ROS_INFO_STREAM("Update current goal " << _current_goal_node);
	switch (_rrt.nodes[_current_goal_node].status) {
	case rrt_nbv_exploration_msgs::Node::EXPLORED:
		ROS_INFO("goal explored");
		_nodes_ordered_by_gain.erase(_current_goal_node);
		update_nodes(_rrt.nodes[_current_goal_node]);
		_current_goal_node = *_nodes_ordered_by_gain.rbegin();
		break;
	case rrt_nbv_exploration_msgs::Node::ABORTED: //update all nodes? around robot?
		ROS_INFO("goal aborted");
		_nodes_ordered_by_gain.erase(_current_goal_node);
		update_nodes(_rrt.nodes[_current_goal_node]);
		_gain_calculator->calculate_gain(_rrt.nodes[_current_goal_node],
				_octree);
		_nodes_ordered_by_gain.insert(_current_goal_node);
		_current_goal_node = *_nodes_ordered_by_gain.rbegin();
		break;
	case rrt_nbv_exploration_msgs::Node::FAILED:
		ROS_INFO("goal failed");
		_nodes_ordered_by_gain.erase(_current_goal_node);
		update_nodes(_rrt.nodes[_current_goal_node]);
		//erase failed node from tree
		_current_goal_node = *_nodes_ordered_by_gain.rbegin();
		break;
	default:    //active or waiting
		break;
	}
	//ROS_INFO("Set current goal to %i", _current_goal_node);
}

void TreeConstructor::convert_octomap_msg_to_octree(
		const octomap_msgs::Octomap::ConstPtr& map_msg) {
	//ROS_INFO_STREAM("Receive octomap");
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	//ROS_INFO_STREAM("first abstract octomap");
	_octree = boost::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
	//ROS_INFO_STREAM("dynamic cast octomap");
	update_map_dimensions();
}

void TreeConstructor::update_map_dimensions() {
	//ROS_INFO_STREAM("Octomap dimensions");
	_octree->getMetricSize(_map_dimensions[0], _map_dimensions[1],
			_map_dimensions[2]);
}

bool TreeConstructor::requestGoal(
		rrt_nbv_exploration_msgs::RequestGoal::Request &req,
		rrt_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.goal = _rrt.nodes[_current_goal_node].position;
	return true;
}

bool TreeConstructor::updateCurrentGoal(
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Response &res) {
	_rrt.nodes[_current_goal_node].status = req.status;
	update_current_goal();
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
			start_rrt_construction();
			res.success = true;
			res.message = "Tree construction started";
		}
	} else {
		if (!_running) {
			res.success = false;
			res.message = "Tree construction already stopped";
		} else {
			stop_rrt_construction();
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
