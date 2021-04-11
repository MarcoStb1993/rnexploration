#include "rrt_nbv_exploration/RneVisualizer.h"

namespace rrt_nbv_exploration {
RneVisualizer::RneVisualizer() {
	ros::NodeHandle nh("rne");
	_frontiers_sub = nh.subscribe("frontiers", 1000,
			&RneVisualizer::visualizeRrtTree, this);
	_frontiers_visualization_pub = nh.advertise<visualization_msgs::Marker>(
			"frontiers_vis", 1000);
	_frontiers_text_info_visualization_pub = nh.advertise<
			visualization_msgs::MarkerArray>("frontiers_vis_info", 1000);
}

RneVisualizer::~RneVisualizer() {

}

void RneVisualizer::initializeVisualization() {
	_frontiers.header.frame_id = "/map";
	_frontiers.ns = "rrt_tree";
	_frontiers.id = 0;
	_frontiers.action = visualization_msgs::Marker::ADD;
	_frontiers.pose.orientation.w = 1.0;
	_frontiers.type = visualization_msgs::Marker::SPHERE_LIST;
	_frontiers.scale.x = 0.2f;
	_frontiers.scale.y = 0.2f;
	_frontiers.scale.z = 0.2f;
	_frontiers.color.g = 1.0f;
	_frontiers.color.a = 1.0f;
}

void RneVisualizer::visualizeRrtTree(
		const rrt_nbv_exploration_msgs::Frontiers::ConstPtr &frontiers) {
	_frontiers.header.stamp = ros::Time::now();
	_frontiers.points.clear();
	_frontiers.colors.clear();
	_frontier_info_texts.markers.clear();
	for (int i = 0; i < frontiers->frontier_counter; i++) {
		_frontiers.points.push_back(frontiers->frontiers[i].centroid);
		std_msgs::ColorRGBA color;
		color.a = 1.0f;
		if (frontiers->frontiers[i].gain == -1) {
			color.r = 0.9f;
			color.g = 0.9f;
			color.b = 0.9f;
		} else {
			switch (frontiers->frontiers[i].status) {
			case rrt_nbv_exploration_msgs::Frontier::EXPLORED:
				color.g = 0.6f;
				break;
			case rrt_nbv_exploration_msgs::Frontier::VISITED:
				color.g = 1.0f;
				break;
			case rrt_nbv_exploration_msgs::Frontier::FAILED:
				color.r = 1.0f;
				break;
			case rrt_nbv_exploration_msgs::Frontier::ACTIVE_VISITED:
				color.r = 1.0f;
				color.g = 0.6f;
				break;
			case rrt_nbv_exploration_msgs::Frontier::ACTIVE:
				color.r = 1.0f;
				color.g = 1.0f;
				break;
			default:
				color.b = 1.0f;
				break;
			}
		}
		_frontiers.colors.push_back(color);
		addInfoTextVisualization(frontiers->frontiers[i].centroid, i,
				frontiers->frontiers[i].gain);
	}
	_frontiers_visualization_pub.publish(_frontiers);
	_frontiers_text_info_visualization_pub.publish(_frontier_info_texts);
}

void RneVisualizer::addInfoTextVisualization(
		const geometry_msgs::Point frontier_position, int frontier,
		double gain) {
	visualization_msgs::Marker frontier_info_text;
	frontier_info_text.header.frame_id = "/map";
	frontier_info_text.ns = "frontiers";
	frontier_info_text.id = frontier;
	frontier_info_text.action = visualization_msgs::Marker::ADD;
	frontier_info_text.pose.orientation.w = 1.0;
	frontier_info_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	frontier_info_text.scale.z = 0.5f;
	frontier_info_text.color.r = 1.0f;
	frontier_info_text.color.g = 1.0f;
	frontier_info_text.color.b = 1.0f;
	frontier_info_text.color.a = 1.0f;
	frontier_info_text.pose.position.x = frontier_position.x;
	frontier_info_text.pose.position.y = frontier_position.y;
	frontier_info_text.pose.position.z = frontier_position.z + 0.5;
	std::ostringstream oss;
	oss << "(" << frontier << ")" << std::setprecision(4) << gain;
	frontier_info_text.text = oss.str();
	_frontier_info_texts.markers.push_back(frontier_info_text);
}
}
