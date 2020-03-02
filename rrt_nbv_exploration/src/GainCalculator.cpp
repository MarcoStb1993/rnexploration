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
	ros::NodeHandle nh("rne");
	_raycast_visualization = nh.advertise<visualization_msgs::Marker>(
			"raycast_visualization", 1000);
	precalculateGainPollPoints();
}

GainCalculator::~GainCalculator() {
	_gain_poll_points.resize(boost::extents[0][0][0]);
}

void GainCalculator::precalculateGainPollPoints() {
	std::vector<StepStruct> theta_steps;
	std::vector<StepStruct> phi_steps;
	ROS_INFO_STREAM("Theta steps:");
	for (int theta = 0; theta <= 360; theta += _delta_theta) {
		double rad = theta * M_PI / 180;
		StepStruct step = { theta, cos(rad), sin(rad) };
		theta_steps.push_back(step);
		ROS_INFO_STREAM(rad);
	}
	ROS_INFO_STREAM("Phi steps:");
	for (int phi = -_sensor_vertical_fov / 2; phi <= _sensor_vertical_fov / 2;
			phi += _delta_phi) {
		double rad = phi * M_PI / 180;
		StepStruct step = { phi, cos(rad), sin(rad) };
		phi_steps.push_back(step);
		ROS_INFO_STREAM(rad);
	}
	int radius_steps = (int) ((_sensor_max_range - _sensor_min_range)
			/ _delta_radius);
	ROS_INFO_STREAM(
			"Dimensions: " << theta_steps.size( )<< ", "<< phi_steps.size()<< ", "<< radius_steps);
	_gain_poll_points.resize(
			boost::extents[theta_steps.size()][phi_steps.size()][radius_steps]);
	for (int t = 0; t < theta_steps.size(); t++) {
		StepStruct theta = theta_steps.at(t);
		for (int p = 0; p < phi_steps.size(); p++) {
			StepStruct phi = phi_steps.at(p);
			for (int r = 0; r < radius_steps; r++) {
				double radius = _sensor_min_range + r * _delta_radius;
				_gain_poll_points[t][p][r] = {radius * theta.cos * phi.sin, radius
					* theta.sin * phi.sin, radius * phi.cos, theta.step,
					phi.step, radius};
			}
		}
	}
}

void GainCalculator::calculate_gain(rrt_nbv_exploration_msgs::Node &node,
		boost::shared_ptr<octomap::OcTree> octree) {
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

	node.gain = 0;

	double x = node.position.x;
	double y = node.position.y;
	double z = node.position.z;

	int overlap = 0;
	std::map<int, double> gain_per_yaw;
	ROS_INFO_STREAM("Raycasting:");
	for (multi_array_index theta = 0; theta < _gain_poll_points.shape()[0];
			theta++) {
//		ROS_INFO_STREAM(
		//"Theta angle: " << _gain_poll_points[theta][0][0].theta);
		for (multi_array_index phi = 0; phi < _gain_poll_points.shape()[1];
				phi++) {
//			ROS_INFO_STREAM(
//					"Phi angle: " << _gain_poll_points[theta][phi][0].phi);
			int raygain = 0;
			for (multi_array_index radius = 0;
					radius < _gain_poll_points.shape()[2]; radius++) {
				PollPoint point = _gain_poll_points[theta][phi][radius];
				geometry_msgs::Point vis_point;
				vis_point.x = point.x + x;
				vis_point.y = point.y + y;
				vis_point.z = point.z + z;
				octomap::OcTreeNode* ocnode = octree->search(vis_point.x,
						vis_point.y, vis_point.z);
				std_msgs::ColorRGBA color;
				color.r = 0.0f;
				color.g = 0.0f;
				color.b = 1.0f;
				color.a = 1.0f;
				_node_points.points.push_back(vis_point);
				if (ocnode != NULL) {
					if (octree->isNodeOccupied(ocnode)) {
						color.r = 1.0f;
						color.b = 0.0f;
						//break; //end raycast for this ray because of an obstacle in the way
					} else {
						color.g = 1.0f;
						color.b = 0.0f;
					}
				} else {
					raygain++;
				}
				_node_points.colors.push_back(color);
			}
			node.gain += (float) raygain;
		}
	}

	ROS_INFO("Gain: %i", (int )node.gain);
	if (_visualize_gain_calculation) {
		_raycast_visualization.publish(_node_points);
	}
}
}
