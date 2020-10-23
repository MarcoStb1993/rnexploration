#include "rrt_nbv_exploration/GainCalculator.h"

namespace rrt_nbv_exploration {

GainCalculator::GainCalculator() :
		_gain_poll_points(boost::extents[0][0][0]) {
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_max_range", _sensor_max_range, 5.0);
	private_nh.param("sensor_min_range", _sensor_min_range, 5.0);
	private_nh.param("delta_phi", _delta_phi, 10);
	private_nh.param("delta_theta", _delta_theta, 10);
	private_nh.param("delta_radius", _delta_radius, 0.1);
	private_nh.param("sensor_horizontal_fov", _sensor_horizontal_fov, 360);
	private_nh.param("sensor_vertical_fov", _sensor_vertical_fov, 180);
	private_nh.param("visualize_gain_calculation", _visualize_gain_calculation,
			false);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	private_nh.param("min_view_score", _min_view_score, 0.1);
	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	ros::NodeHandle nh("rne");
	_raycast_visualization = nh.advertise<visualization_msgs::Marker>(
			"raycast_visualization", 1000);
	_updated_node_publisher = nh.advertise<rrt_nbv_exploration_msgs::Node>(
			"updated_node", 1);
	_node_to_update_subscriber = nh.subscribe("node_to_update", 1,
			&GainCalculator::nodeToUpdateCallback, this);

	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&GainCalculator::convertOctomapMsgToOctree, this);

	_best_gain_per_view = 0;
	_max_gain_points = 0;
	_last_updated_node.index = -1;
}

GainCalculator::~GainCalculator() {
	_gain_poll_points.resize(boost::extents[0][0][0]);
}

void GainCalculator::precalculateGainPollPoints() {
	std::vector<StepStruct> theta_steps;
	std::vector<StepStruct> phi_steps;
	for (int theta = 0; theta <= 360; theta += _delta_theta) {
		double rad = theta * M_PI / 180;
		StepStruct step = { theta, cos(rad), sin(rad) };
		theta_steps.push_back(step);
	}
	for (int phi = -_sensor_vertical_fov / 2; phi <= _sensor_vertical_fov / 2;
			phi += _delta_phi) {
		double rad = (phi + 90) * M_PI / 180;
		StepStruct step = { phi, cos(rad), sin(rad) };
		phi_steps.push_back(step);
	}
	int radius_steps = (int) ((_sensor_max_range - _sensor_min_range)
			/ _delta_radius);
	_gain_poll_points.resize(
			boost::extents[theta_steps.size()][phi_steps.size()][radius_steps]);
	for (int t = 0; t < theta_steps.size(); t++) {
		StepStruct theta = theta_steps.at(t);
		for (int p = 0; p < phi_steps.size(); p++) {
			StepStruct phi = phi_steps.at(p);
			for (int r = 0; r < radius_steps; r++) {
				double radius = _sensor_min_range + r * _delta_radius;
				_gain_poll_points[t][p][r] = { radius * theta.cos * phi.sin,
						radius * theta.sin * phi.sin, radius * phi.cos,
						theta.step, phi.step, radius };
			}
		}
	}

	_best_gain_per_view = phi_steps.size() * radius_steps
			* (_sensor_horizontal_fov / _delta_theta + 1);
	_max_gain_points = theta_steps.size() * phi_steps.size() * radius_steps;
	ROS_INFO_STREAM(
			"Gain calculation initialized with " << _max_gain_points << " poll points and max reachable gain per view of " << _best_gain_per_view);
}

void GainCalculator::calculateGain(rrt_nbv_exploration_msgs::Node &node) {
	visualization_msgs::Marker _node_points;
	_node_points.header.frame_id = "/map";
	_node_points.ns = "raycast_visualization";
	_node_points.id = 0;
	_node_points.action = visualization_msgs::Marker::ADD;
	_node_points.pose.orientation.w = 1.0;
	_node_points.type = visualization_msgs::Marker::SPHERE_LIST;
	_node_points.scale.x = 0.1;
	_node_points.scale.y = 0.1;
	_node_points.scale.z = 0.1;
	_node_points.color.a = 1.0f;
	_node_points.header.stamp = ros::Time::now();

	node.gain = 0.0;

	std::map<int, int> gain_per_yaw;

	if (node.index != 0 && !measureNodeHeight(node)) {
		node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
		node.gain = 0;
		return;
	}

	double x = node.position.x;
	double y = node.position.y;
	double z = node.position.z;

	for (multi_array_index theta = 0; theta < _gain_poll_points.shape()[0];
			theta++) {
		for (multi_array_index phi = 0; phi < _gain_poll_points.shape()[1];
				phi++) {
			for (multi_array_index radius = 0;
					radius < _gain_poll_points.shape()[2]; radius++) {
				PollPoint point = _gain_poll_points[theta][phi][radius];
				geometry_msgs::Point vis_point;
				vis_point.x = point.x + x;
				vis_point.y = point.y + y;
				vis_point.z = point.z + z;
				octomap::OcTreeNode *ocnode = _octree->search(vis_point.x,
						vis_point.y, vis_point.z);
				std_msgs::ColorRGBA color;
				color.r = 0.0f;
				color.g = 0.0f;
				color.b = 1.0f;
				color.a = 1.0f;
				_node_points.points.push_back(vis_point);
				if (ocnode != NULL) {
					if (_octree->isNodeOccupied(ocnode)) {
						color.r = 1.0f;
						color.b = 0.0f;
						_node_points.colors.push_back(color);
						break; //end raycast for this ray because of an obstacle in the way
					} else {
						color.g = 1.0f;
						color.b = 0.0f;
					}
				} else {
					gain_per_yaw[point.theta]++;
				}
				_node_points.colors.push_back(color);
			}
		}
	}

	int best_yaw = 0;
	int best_yaw_score = 0;
	for (int yaw = 0; yaw < 360; yaw++) {
		double yaw_score = 0;
		for (int fov = -_sensor_horizontal_fov / 2;
				fov < _sensor_horizontal_fov / 2; fov++) {
			int theta = yaw + fov;
			if (theta < 0)
				theta += 360;
			if (theta >= 360)
				theta -= 360;
			yaw_score += gain_per_yaw[theta];
		}
		if (best_yaw_score < yaw_score) {
			best_yaw_score = yaw_score;
			best_yaw = yaw;
		}
	}

	double view_score = (double) best_yaw_score / (double) _best_gain_per_view;

	if (view_score < _min_view_score
			|| (node.status == rrt_nbv_exploration_msgs::Node::VISITED
					&& node.best_yaw <= best_yaw + 5
					&& node.best_yaw >= best_yaw - 5)) {
		//no use exploring similar yaw again, sensor position approximation flawed in this case
		node.status = rrt_nbv_exploration_msgs::Node::EXPLORED;
		node.gain = 0;
	} else {
		node.gain = best_yaw_score;
		node.best_yaw = best_yaw;
	}

	//Visualize best yaw direction
	geometry_msgs::Point vis_point;
	vis_point.x = x
			+ (_sensor_max_range + _delta_radius)
					* cos(M_PI * best_yaw / 180.0);
	vis_point.y = y
			+ (_sensor_max_range + _delta_radius)
					* sin(M_PI * best_yaw / 180.0);
	vis_point.z = z;
	std_msgs::ColorRGBA color;
	color.r = 1.0f;
	color.g = 1.0f;
	color.b = 0.0f;
	color.a = 1.0f;
	_node_points.points.push_back(vis_point);
	_node_points.colors.push_back(color);

	if (_visualize_gain_calculation) {
		_raycast_visualization.publish(_node_points);
	}
}

bool GainCalculator::measureNodeHeight(rrt_nbv_exploration_msgs::Node &node) {
	double map_x, map_y, map_z;
	_octree->getMetricMin(map_x, map_y, map_z);
	octomap::point3d node_point(node.position.x, node.position.y,
			node.position.z);
	octomap::point3d bottom_point(node.position.x, node.position.y, map_z);
	octomap::KeyRay keyray;
	// Raytrace from initial node height (parent height) to min z value to find first occupied voxel
	if (_octree->computeRayKeys(node_point, bottom_point, keyray)) {
		for (auto iterator : keyray) {
			octomap::point3d coords = _octree->keyToCoord(iterator);
			octomap::OcTreeNode *ocnode = _octree->search(iterator);
			if (ocnode != NULL) {
				if (_octree->isNodeOccupied(ocnode)) {
					node.position.z = coords.z() + _sensor_height;
					return true;
				}
			}
		}
	} else {
		ROS_INFO_STREAM(
				"Raytracing for node height measurement to min z out of bounds");
	}
	_octree->getMetricMin(map_x, map_y, map_z);
	octomap::point3d top_point(node.position.x, node.position.y, map_z);
	// Raytrace from initial node height (parent height) to max z value to find first free voxel
	if (_octree->computeRayKeys(node_point, top_point, keyray)) {
		for (auto iterator : keyray) {
			octomap::point3d coords = _octree->keyToCoord(iterator);
			octomap::OcTreeNode *ocnode = _octree->search(iterator);
			if (ocnode != NULL) {
				if (!_octree->isNodeOccupied(ocnode)) {
					node.position.z = coords.z() + _sensor_height;
					return true;
				}
			}
		}
	} else {
		ROS_INFO_STREAM(
				"Raytracing for node height measurement to max z out of bounds");
	}
	ROS_INFO_STREAM("Could not determine height for node " << node.index);
	return false;
}

void GainCalculator::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr &map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
}

void GainCalculator::nodeToUpdateCallback(
		const rrt_nbv_exploration_msgs::Node::ConstPtr &node_to_update) {
//	ROS_WARN_STREAM("update node: " << node_to_update->index << " previous node: " << _last_updated_node.index);
	if (_last_updated_node.index != node_to_update->index) {
		rrt_nbv_exploration_msgs::Node node;
		geometry_msgs::Point pos;
		pos.x = node_to_update->position.x;
		pos.y = node_to_update->position.y;
		pos.z = node_to_update->position.z;
		node.position = pos;
		node.status = node_to_update->status;
		node.index = node_to_update->index;
		node.gain = node_to_update->gain;
		node.best_yaw = node_to_update->best_yaw;
		calculateGain(node);
		_last_updated_node = node;
		_updated_node_publisher.publish(node);
	} else {
		_updated_node_publisher.publish(_last_updated_node);
	}
}

void GainCalculator::dynamicReconfigureCallback(
		rrt_nbv_exploration::GainCalculatorConfig &config, uint32_t level) {
	_min_view_score = config.min_view_score;
}

}
