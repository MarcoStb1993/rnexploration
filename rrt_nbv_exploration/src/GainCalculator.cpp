#include "rrt_nbv_exploration/GainCalculator.h"

namespace rrt_nbv_exploration {

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
			0);
	private_nh.param("sensor_vertical_fov_top", _sensor_vertical_fov_top, 180);
	private_nh.param("visualize_gain_calculation", _visualize_gain_calculation,
			false);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	private_nh.param("sensor_size", _sensor_size, 0.1);
	private_nh.param("min_view_score", _min_view_score, 0.1);
	private_nh.param("oc_resolution", _octomap_resolution, 0.1);
	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	ros::NodeHandle nh("rne");
	raysample_visualization = nh.advertise<visualization_msgs::Marker>(
			"raysample_visualization", 1000);
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&GainCalculator::convertOctomapMsgToOctree, this);

	_best_gain_per_view = 0;
	_max_gain_points = 0;
	_last_updated_frontier.index = -1;
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
	_max_gain_points = theta_steps.size() * phi_steps.size() * range_steps;
//	ROS_WARN_STREAM(
//			"Ray sampling gain calculation initialized with " << _max_gain_points << " poll points and max reachable gain per view of " << _best_gain_per_view);
}

void GainCalculator::calculateGain(
		rrt_nbv_exploration_msgs::Frontier &frontier) {
	if (!measureFrontierHeight(frontier)) {
		frontier.centroid.z = _sensor_height;
	}
	calculatePointGain(frontier);
}

void GainCalculator::calculatePointGain(
		rrt_nbv_exploration_msgs::Frontier &frontier) {
	visualization_msgs::Marker _frontier_points;
	_frontier_points.header.frame_id = "/map";
	_frontier_points.ns = "raysample_visualization";
	_frontier_points.id = 0;
	_frontier_points.action = visualization_msgs::Marker::ADD;
	_frontier_points.pose.orientation.w = 1.0;
	_frontier_points.type = visualization_msgs::Marker::SPHERE_LIST;
	_frontier_points.scale.x = 0.1;
	_frontier_points.scale.y = 0.1;
	_frontier_points.scale.z = 0.1;
	_frontier_points.color.a = 1.0f;
	_frontier_points.header.stamp = ros::Time::now();

	frontier.gain = 0.0;

	std::map<int, int> gain_per_yaw;

	double x = frontier.centroid.x;
	double y = frontier.centroid.y;
	double z = frontier.centroid.z;

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
				_frontier_points.points.push_back(vis_point);
				if (ocnode != NULL) {
					if (_octree->isNodeOccupied(ocnode)) {
						color.r = 1.0f;
						color.b = 0.0f;
						_frontier_points.colors.push_back(color);
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
				_frontier_points.colors.push_back(color);
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

//	ROS_INFO_STREAM(
//			"Best yaw score: " << best_yaw_score << " view score: " << view_score << " best yaw: " << best_yaw);

	if (view_score < _min_view_score
			|| (frontier.status == rrt_nbv_exploration_msgs::Frontier::VISITED
					&& frontier.best_yaw <= best_yaw + 5
					&& frontier.best_yaw >= best_yaw - 5)) {
		//no use exploring similar yaw again, sensor position approximation flawed in this case
		frontier.status = rrt_nbv_exploration_msgs::Frontier::EXPLORED;
		frontier.gain = 0;
	} else {
		frontier.gain = best_yaw_score;
		frontier.best_yaw = best_yaw;
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
	_frontier_points.points.push_back(vis_point);
	_frontier_points.colors.push_back(color);
	if (_visualize_gain_calculation) {
		raysample_visualization.publish(_frontier_points);
	}
}

bool GainCalculator::measureFrontierHeight(
		rrt_nbv_exploration_msgs::Frontier &frontier) {
	double min_x, min_y, min_z;
	_octree->getMetricMin(min_x, min_y, min_z);
	octomap::point3d frontier_point(frontier.centroid.x, frontier.centroid.y,
			frontier.centroid.z);
	octomap::point3d bottom_point(frontier.centroid.x, frontier.centroid.y,
			min_z - _octomap_resolution); //seems to ignore lowest voxel if using only min_z
	octomap::KeyRay keyray;
// Raytrace from initial frontier height (parent height) to min z value to find first occupied voxel (assume current z pos is above ground)
	if (_octree->computeRayKeys(frontier_point, bottom_point, keyray)) {
		for (auto iterator : keyray) {
			octomap::point3d coords = _octree->keyToCoord(iterator);
			octomap::OcTreeNode *ocnode = _octree->search(iterator);
			if (ocnode != NULL) {
				if (_octree->isNodeOccupied(ocnode)) {
					frontier.centroid.z = coords.z() + _sensor_height;
					return true;
				}
			}
		}
	} else {
		ROS_INFO_STREAM(
				"Raytracing for frontier height measurement to min z out of bounds");
	}
	double max_x, max_y, max_z;
	_octree->getMetricMax(max_x, max_y, max_z);
	octomap::point3d top_point(frontier.centroid.x, frontier.centroid.y,
			max_z + _octomap_resolution); //seems to ignore lowest voxel if using only min_z, suspect the same for max z
// Raytrace from initial frontier height (parent height) to max z value to find first free voxel after occupied voxel (assume current z pos is below ground)
	bool ground_detected = false;
	if (_octree->computeRayKeys(frontier_point, top_point, keyray)) {
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
						frontier.centroid.z = coords.z() + _sensor_height;
						return true;
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

void GainCalculator::dynamicReconfigureCallback(
		rrt_nbv_exploration::GainCalculatorConfig &config, uint32_t level) {
	_min_view_score = config.min_view_score;
}

}
