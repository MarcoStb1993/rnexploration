#include "rrt_nbv_exploration/TreeConstructor.h"

namespace rrt_nbv_exploration {
TreeConstructor::TreeConstructor() :
		_map_dimensions { 0.0, 0.0, 0.0 }, _nodes_ordered_by_gain(
				[this](int node1, int node2) {return _rrt.nodes[node1].gain < _rrt.nodes[node2].gain;}) {
}

void TreeConstructor::initialization(geometry_msgs::Point seed) {
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("rrt_publish_frequency", loop_rate, 20.0);
	private_nh.param("sensor_range", _sensor_range, 5.0);
	_radius_search_range = pow(2 * _sensor_range, 2);
	_loop_rate = new ros::Rate(loop_rate);
	_rrt_publisher = _nh.advertise<rrt_nbv_exploration_msgs::rrt>("rrt_tree",
			1000);
	_octomap_sub = _nh.subscribe("octomap_binary", 1000,
			&TreeConstructor::convert_octomap_msg_to_octree, this);
	_running = false;
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
	_tree_searcher.initialize(_rrt);
	_generator.seed(time(NULL));
	_running = true;
	run_rrt_construction();
}

void TreeConstructor::start_rrt_construction() {
	_running = true;
}

void TreeConstructor::stop_rrt_construction() {
	_running = false;
}

void TreeConstructor::run_rrt_construction() {
	_rrt.header.stamp = ros::Time::now();
	if (_running && _map_dimensions[0] && _map_dimensions[1]
			&& _map_dimensions[2]) {
		geometry_msgs::Point rand_sample = sample_point();
		double min_distance;
		int nearest_node;
		_tree_searcher.find_nearest_neighbour(rand_sample, min_distance,
				nearest_node);
		place_new_node(rand_sample, min_distance, nearest_node);
		publish_node_with_best_gain();
	}
	_rrt_publisher.publish(_rrt);
}

geometry_msgs::Point TreeConstructor::sample_point() {
	//randomly sampled point
	geometry_msgs::Point rand_sample;
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
		return rand_sample;
	} else {
		return sample_point();
	}
}

void TreeConstructor::place_new_node(geometry_msgs::Point rand_sample,
		double min_distance, int nearest_node) {
	//distance metric/kinematic constraints
	rrt_nbv_exploration_msgs::Node node;
	if (_collision_checker.steer(node, _rrt.nodes[nearest_node], rand_sample,
			min_distance, _octree)) {
		_gain_calculator.calculate_gain(node, _octree);
		_rrt.nodes.push_back(node);
		_nodes_ordered_by_gain.insert(_rrt.node_counter);
		_rrt.nodes[nearest_node].children.push_back(_rrt.node_counter);
		_rrt.nodes[nearest_node].children_counter++;
		_rrt.node_counter++;
		_tree_searcher.rebuildIndex(_rrt);
	}
}

void TreeConstructor::publish_node_with_best_gain() {
//change to iterations!
	if (_nodes_ordered_by_gain.size() > 5) //only start exploration after constructing a few nodes to prevent blocking exploration with a suboptimal goal
			{
		_navigator.check_current_goal_status(_rrt.nodes[_current_goal_node]);
		switch (_rrt.nodes[_current_goal_node].status) {
		case rrt_nbv_exploration_msgs::Node::EXPLORED:
			ROS_INFO("goal explored");
			_nodes_ordered_by_gain.erase(_current_goal_node);
			update_nodes(_rrt.nodes[_current_goal_node]);
			update_current_goal();
			break;
		case rrt_nbv_exploration_msgs::Node::ABORTED: //update all nodes? around robot?
			ROS_INFO("goal aborted");
			_nodes_ordered_by_gain.erase(_current_goal_node);
			update_nodes(_rrt.nodes[_current_goal_node]);
			_gain_calculator.calculate_gain(_rrt.nodes[_current_goal_node],
					_octree);
			_nodes_ordered_by_gain.insert(_current_goal_node);
			update_current_goal();
			break;
		case rrt_nbv_exploration_msgs::Node::FAILED:
			ROS_INFO("goal failed");
			_nodes_ordered_by_gain.erase(_current_goal_node);
			update_nodes(_rrt.nodes[_current_goal_node]);
			//erase failed node from tree
			update_current_goal();
			break;
		default:    //active or waiting

			break;
		}
		/**
		 _navigator.check_current_goal_status(_rrt.nodes[_current_goal_node]);
		 if(_rrt.nodes[_current_goal_node].status != rrt_nbv_exploration_msgs::Node::ACTIVE &&
		 _rrt.nodes[_current_goal_node].status != rrt_nbv_exploration_msgs::Node::WAITING)
		 {
		 _nodes_ordered_by_gain.erase(_current_goal_node);
		 // handle node status

		 }
		 if(_rrt.nodes[_current_goal_node].status != rrt_nbv_exploration_msgs::Node::ACTIVE)
		 {
		 _navigator.publish_nav_goal(_rrt.nodes[*_nodes_ordered_by_gain.rbegin()]);
		 _current_goal_node = *_nodes_ordered_by_gain.rbegin();
		 }
		 **/
	}
}

void TreeConstructor::update_nodes(
		rrt_nbv_exploration_msgs::Node& center_node) {
	ROS_INFO("start updating nodes");
	std::vector<int> updatable_nodes = _tree_searcher.search_in_radius(
			center_node.position, _radius_search_range);
	for (auto iterator : updatable_nodes) {
		ROS_INFO("update node %i", iterator);
		_gain_calculator.calculate_gain(_rrt.nodes[iterator], _octree);
	}
}

void TreeConstructor::update_current_goal() {
	_last_goal_node = _current_goal_node;
	_current_goal_node = *_nodes_ordered_by_gain.rbegin();
	ROS_INFO("Set current goal to %i", _current_goal_node);
	_navigator.publish_nav_goal(_rrt.nodes[_current_goal_node]);
}

void TreeConstructor::convert_octomap_msg_to_octree(
		const octomap_msgs::Octomap::ConstPtr& map_msg) {
	_abstract_octree = octomap_msgs::msgToMap(*map_msg);
	_octree = dynamic_cast<octomap::OcTree*>(_abstract_octree);
	update_map_dimensions();
}

void TreeConstructor::update_map_dimensions() {
	_octree->getMetricSize(_map_dimensions[0], _map_dimensions[1],
			_map_dimensions[2]);
}
}
