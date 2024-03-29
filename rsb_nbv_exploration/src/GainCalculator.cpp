#include <rsb_nbv_exploration/GainCalculator.h>

namespace rsb_nbv_exploration {

GainCalculator::GainCalculator() :
		_gain_poll_points(boost::extents[0][0][0]) {
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_max_range", _sensor_max_range, 5.0);
	private_nh.param("sensor_min_range", _sensor_min_range, 1.0);
	private_nh.param("delta_phi", _delta_phi, 10);
	private_nh.param("delta_theta", _delta_theta, 10);
	private_nh.param("delta_radius", _delta_radius, 0.1);
	private_nh.param("sensor_horizontal_fov", _sensor_horizontal_fov, 360);
	private_nh.param("sensor_vertical_fov_bottom", _sensor_vertical_fov_bottom,
			180);
	private_nh.param("sensor_vertical_fov_top", _sensor_vertical_fov_top, 0);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	private_nh.param("sensor_size", _sensor_size, 0.1);
	private_nh.param("min_view_score", _min_view_score, 0.1);
	private_nh.param("oc_resolution", _octomap_resolution, 0.1);
	private_nh.param("max_node_height_difference", _max_node_height_difference,
			1.0);
	private_nh.param("measure_algorithm_runtime", _measure_algorithm_runtime,
			false);
	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	ros::NodeHandle nh("rne");
	raysample_visualization = nh.advertise<visualization_msgs::Marker>(
			"raysample_visualization", 1000);
	_updated_node_publisher = nh.advertise<rsb_nbv_exploration_msgs::Node>(
			"updated_node", 1);
	_node_to_update_subscriber = nh.subscribe("node_to_update", 1,
			&GainCalculator::nodeToUpdateCallback, this);
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&GainCalculator::convertOctomapMsgToOctree, this);

	if (_measure_algorithm_runtime) {
		_gaincalc_runtime_publisher = nh.advertise<std_msgs::Duration>(
				"gaincalc_runtime", 1);
		_rne_runtime_subscriber = nh.subscribe("rne_runtime", 1,
				&GainCalculator::rneRuntimeCallback, this);
		_algorithm_runtime = ros::Duration(0, 0);
	}

	if (_sensor_vertical_fov_top > _sensor_vertical_fov_bottom) {
		ROS_ERROR_STREAM(
				"Sensor vertical FoV top must be smaller than bottom! Straight up is 0 degrees and down is 180 degrees. They will be reversed now");
		double tmp = _sensor_vertical_fov_bottom;
		_sensor_vertical_fov_bottom = _sensor_vertical_fov_top;
		_sensor_vertical_fov_top = tmp;
	}
	_best_gain_per_view = 0;
	_last_updated_node.index = -1;
	_sensor_min_range_squared = pow(_sensor_min_range, 2);
}

GainCalculator::~GainCalculator() {
	_gain_poll_points.resize(boost::extents[0][0][0]);
}

void GainCalculator::precalculateGainPolls() {
	precalculateGainPollPoints();
}

void GainCalculator::precalculateGainPollPoints() {
	std::vector<StepStruct> theta_steps;
	std::vector<StepStruct> phi_steps;
	for (int theta = 0; theta < 360; theta += _delta_theta) {
		double rad = theta * M_PI / 180;
		StepStruct step = { theta, cos(rad), sin(rad) };
		theta_steps.push_back(step);
	}
	for (int phi = _sensor_vertical_fov_top; phi <= _sensor_vertical_fov_bottom;
			phi += _delta_phi) {
		double rad = phi * M_PI / 180;
		StepStruct step = { phi, cos(rad), sin(rad) };
		phi_steps.push_back(step);
	}
	int radius_steps =
			(int) ((_sensor_max_range - _sensor_size) / _delta_radius);
	_gain_poll_points.resize(
			boost::extents[theta_steps.size()][phi_steps.size()][radius_steps]);
	for (int t = 0; t < theta_steps.size(); t++) {
		StepStruct theta = theta_steps.at(t);
		for (int p = 0; p < phi_steps.size(); p++) {
			StepStruct phi = phi_steps.at(p);
			for (int r = 0; r < radius_steps; r++) {
				double radius = _sensor_size + r * _delta_radius;
				_gain_poll_points[t][p][r] =
						{ radius * theta.cos * phi.sin, radius * theta.sin
								* phi.sin, radius * phi.cos, theta.step,
								phi.step, radius, radius > _sensor_min_range };
			}
		}
	}

	int range_steps = (int) ((_sensor_max_range - _sensor_min_range)
			/ _delta_radius);
	_best_gain_per_view = phi_steps.size() * range_steps
			* (_sensor_horizontal_fov / _delta_theta + 1);
}

void GainCalculator::calculateGain(rsb_nbv_exploration_msgs::Node &node) {
	ros::Time start_time = ros::Time::now();
	if (!measureNodeHeight(node) && node.distance_to_robot != 0
			&& node.edges.size() < 2) {
		//node.status = rsb_nbv_exploration_msgs::Node::INITIAL;
		node.gain = -1;
		return;
	}
	calculatePointGain(node);
	if (_measure_algorithm_runtime) {
		_algorithm_runtime += ros::Time::now() - start_time;
	}
}

void GainCalculator::calculatePointGain(rsb_nbv_exploration_msgs::Node &node) {
	bool publish_visualization = raysample_visualization.getNumSubscribers()
			> 0;
	visualization_msgs::Marker _node_points;
	if (publish_visualization) {
		_node_points.header.frame_id = "map";
		_node_points.ns = "raysample_visualization";
		_node_points.id = 0;
		_node_points.action = visualization_msgs::Marker::ADD;
		_node_points.pose.orientation.w = 1.0;
		_node_points.type = visualization_msgs::Marker::SPHERE_LIST;
		_node_points.scale.x = 0.1;
		_node_points.scale.y = 0.1;
		_node_points.scale.z = 0.1;
		_node_points.color.a = 1.0f;
		_node_points.header.stamp = ros::Time::now();
	}
	node.gain = 0.0;

	std::map<int, int> gain_per_yaw;

	double min_x, min_y, min_z;
	_octree->getMetricMin(min_x, min_y, min_z);
	double max_x, max_y, max_z;
	_octree->getMetricMax(max_x, max_y, max_z);

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
				if (vis_point.z >= max_z || vis_point.z <= min_z
						|| vis_point.x >= max_x || vis_point.x <= min_x
						|| vis_point.y >= max_y || vis_point.y <= min_y) { //ray out of OctoMap bounds
					if (!point.in_range) { //find first point above gain range
						for (multi_array_index r = radius;
								r < _gain_poll_points.shape()[2]; r++) {
							if (_gain_poll_points[theta][phi][r].in_range) {
								radius = r;
								break;
							}
						}
					}
					gain_per_yaw[point.theta] += (_gain_poll_points.shape()[2]
							- radius); //add all remaining points on ray to gain
					break;
				}
				if (publish_visualization) {
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
							break; //end ray sampling for this ray because of an obstacle in the way
						} else {
							color.g = 1.0f;
							color.b = 0.0f;
						}
					} else {
						if (point.in_range)
							gain_per_yaw[point.theta]++;
						else {
							color.r = 0.7f;
							color.g = 0.7f;
							color.b = 0.7f;
						}

					}
					_node_points.colors.push_back(color);
				} else {
					octomap::OcTreeNode *ocnode = _octree->search(vis_point.x,
							vis_point.y, vis_point.z);
					if (ocnode != NULL) {
						if (_octree->isNodeOccupied(ocnode))
							break; //end ray sampling for this ray because of an obstacle in the way
					} else if (point.in_range)
						gain_per_yaw[point.theta]++;
				}
			}
		}
	}

	int best_yaw = 0;
	int best_yaw_score = 0;
	int horizontal_fov =
			_sensor_horizontal_fov == 360 ? 270 : _sensor_horizontal_fov; //get a best yaw for 360deg sensors (always 0 otherwise)

	for (int yaw = 0; yaw < 360; yaw++) {
		double yaw_score = 0;
		for (int fov = -horizontal_fov / 2; fov < horizontal_fov / 2; fov++) {
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
			|| (node.status == rsb_nbv_exploration_msgs::Node::VISITED
					&& node.best_yaw <= best_yaw + 5
					&& node.best_yaw >= best_yaw - 5)) {
		//no use exploring similar yaw again, sensor position approximation flawed in this case
		node.status = rsb_nbv_exploration_msgs::Node::EXPLORED;
		node.gain = 0;
	} else {
		node.gain = view_score;
		node.best_yaw = best_yaw;
	}

	if (publish_visualization) {
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
		raysample_visualization.publish(_node_points);
	}

}

bool GainCalculator::measureNodeHeight(rsb_nbv_exploration_msgs::Node &node) {
	double min_x, min_y, min_z;
	_octree->getMetricMin(min_x, min_y, min_z);
	octomap::point3d node_point(node.position.x, node.position.y,
			node.position.z);
	octomap::point3d bottom_point(node.position.x, node.position.y,
			min_z - _octomap_resolution); //seems to ignore lowest voxel if using only min_z
	octomap::KeyRay keyray;
// Raytrace from initial node height (parent height) to min z value to find first occupied voxel (assume current z pos is above ground)
	if (_octree->computeRayKeys(node_point, bottom_point, keyray)) {
		for (auto iterator : keyray) {
			octomap::point3d coords = _octree->keyToCoord(iterator);
			octomap::OcTreeNode *ocnode = _octree->search(iterator);
			if (ocnode != NULL) {
				if (_octree->isNodeOccupied(ocnode)) {
					double new_height = coords.z() + _sensor_height;
					if (abs(node.position.z - new_height)
							<= _max_node_height_difference) {
						node.position.z = new_height;
						return true;
					}
				}
			}
		}
	} else {
		ROS_INFO_STREAM(
				"Raytracing for node height measurement to min z out of bounds");
	}
	double max_x, max_y, max_z;
	_octree->getMetricMax(max_x, max_y, max_z);
	octomap::point3d top_point(node.position.x, node.position.y,
			max_z + _octomap_resolution); //seems to ignore lowest voxel if using only min_z, suspect the same for max z
// Raytrace from initial node height (parent height) to max z value to find first free voxel after occupied voxel (assume current z pos is below ground)
	bool ground_detected = false;
	if (_octree->computeRayKeys(node_point, top_point, keyray)) {
		for (auto iterator : keyray) {
			octomap::point3d coords = _octree->keyToCoord(iterator);
			octomap::OcTreeNode *ocnode = _octree->search(iterator);
			if (ocnode != NULL) {
				if (!ground_detected) {	//find first occupied voxel
					if (_octree->isNodeOccupied(ocnode)) {
						ground_detected = true;
					}
				} else {//find first empty voxel above ground, if there are none, it was probably the ceiling
					if (!_octree->isNodeOccupied(ocnode)) {
						double new_height = coords.z() + _sensor_height;
						if (abs(node.position.z - new_height)
								<= _max_node_height_difference) {
							node.position.z = new_height;
							return true;
						}
					}
				}
			}
		}
	} else {
		ROS_INFO_STREAM(
				"Raytracing for node height measurement to max z out of bounds");
	}
	return false;
}

void GainCalculator::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr &map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
}

void GainCalculator::nodeToUpdateCallback(
		const rsb_nbv_exploration_msgs::NodeToUpdate::ConstPtr &node_to_update) {
	if (_last_updated_node.index != node_to_update->node.index
			|| node_to_update->force_update) {
		rsb_nbv_exploration_msgs::Node node;
		geometry_msgs::Point pos;
		pos.x = node_to_update->node.position.x;
		pos.y = node_to_update->node.position.y;
		pos.z = node_to_update->node.position.z;
		node.position = pos;
		node.status = node_to_update->node.status;
		node.index = node_to_update->node.index;
		node.gain = node_to_update->node.gain;
		node.best_yaw = node_to_update->node.best_yaw;
		calculateGain(node);
		_last_updated_node = node;
		_updated_node_publisher.publish(node);
	} else {
		_updated_node_publisher.publish(_last_updated_node);
	}
}

void GainCalculator::rneRuntimeCallback(
		const std_msgs::Duration::ConstPtr &runtime_msg) {
	if (runtime_msg->data.sec == 0 && runtime_msg->data.nsec == 0) { //algorithm not running
		_algorithm_runtime = ros::Duration(0, 0);
	}
	std_msgs::Duration runtime;
	runtime.data = _algorithm_runtime;
	_gaincalc_runtime_publisher.publish(runtime);
}

void GainCalculator::dynamicReconfigureCallback(
		rsb_nbv_exploration::GainCalculatorConfig &config, uint32_t level) {
	_min_view_score = config.min_view_score;
}

}
