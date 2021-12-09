#include <rrg_nbv_exploration/RneVisualizer.h>

namespace rrg_nbv_exploration {
RneVisualizer::RneVisualizer() {
	ros::NodeHandle nh("rne");
	_rrg_sub = nh.subscribe("rrg", 1000, &RneVisualizer::visualizeRrgGraph,
			this);
	_rrg_visualization_pub = nh.advertise<visualization_msgs::Marker>("rrg_vis",
			1000);
	_rrg_text_info_visualization_pub = nh.advertise<
			visualization_msgs::MarkerArray>("rrg_vis_info", 1000);
	_info_interval = 5.0;
}

RneVisualizer::~RneVisualizer() {

}

void RneVisualizer::initializeVisualization(
		visualization_msgs::Marker &_node_points,
		visualization_msgs::Marker &_edge_line_list) {
	_node_points.header.frame_id = "/map";
	_node_points.ns = "rrg";
	_node_points.id = 0;
	_node_points.action = visualization_msgs::Marker::ADD;
	_node_points.pose.orientation.w = 1.0;
	_node_points.type = visualization_msgs::Marker::SPHERE_LIST;
	_node_points.scale.x = 0.2f;
	_node_points.scale.y = 0.2f;
	_node_points.scale.z = 0.2f;
	_node_points.color.g = 1.0f;
	_node_points.color.a = 1.0f;
	_edge_line_list.header.frame_id = "/map";
	_edge_line_list.header.stamp = ros::Time::now();
	_edge_line_list.ns = "rrg";
	_edge_line_list.id = 1;
	_edge_line_list.action = visualization_msgs::Marker::ADD;
	_edge_line_list.pose.orientation.w = 1.0;
	_edge_line_list.type = visualization_msgs::Marker::LINE_LIST;
	_edge_line_list.scale.x = 0.08f;
	_edge_line_list.color.b = 1.0f;
	_edge_line_list.color.a = 1.0f;
}

void RneVisualizer::visualizeRrgGraph(
		const rrg_nbv_exploration_msgs::Graph::ConstPtr &rrg) {
	if (_rrg_visualization_pub.getNumSubscribers() > 0) {
		bool publishInfo = _rrg_text_info_visualization_pub.getNumSubscribers()
				> 0
				&& (ros::Time::now() - _last_info_publish).toSec()
						>= _info_interval;
		visualization_msgs::Marker _node_points;
		visualization_msgs::Marker _edge_line_list;
		initializeVisualization(_node_points, _edge_line_list);
		visualization_msgs::MarkerArray _node_info_texts;
		_node_points.header.stamp = ros::Time::now();
		_node_points.points.clear();
		_node_points.colors.clear();
		_edge_line_list.points.clear();
		_edge_line_list.colors.clear();
		for (int i = 0; i < rrg->node_counter; i++) {
			_node_points.points.push_back(rrg->nodes[i].position);
			_node_points.colors.push_back(getColor(rrg->nodes[i]));
			if (publishInfo)
				addInfoTextVisualization(_node_info_texts,
						rrg->nodes[i].position, i,
						rrg->nodes[i].reward_function);
		}
		for (int j = 0; j < rrg->edges.size(); j++) {
			_edge_line_list.points.push_back(
					rrg->nodes[rrg->edges[j].first_node].position);
			_edge_line_list.colors.push_back(
					getColor(rrg->nodes[rrg->edges[j].first_node]));
			_edge_line_list.points.push_back(
					rrg->nodes[rrg->edges[j].second_node].position);
			_edge_line_list.colors.push_back(
					getColor(rrg->nodes[rrg->edges[j].second_node]));
		}
		_rrg_visualization_pub.publish(_node_points);
		_rrg_visualization_pub.publish(_edge_line_list);
		if (publishInfo) {
			_rrg_text_info_visualization_pub.publish(_node_info_texts);
			_last_info_publish = ros::Time::now();
		}
	}
}

void RneVisualizer::addInfoTextVisualization(
		visualization_msgs::MarkerArray &_node_info_texts,
		const geometry_msgs::Point node_position, int node,
		double reward_function) {
	visualization_msgs::Marker node_info_text;
	node_info_text.header.frame_id = "/map";
	node_info_text.ns = "rrt_tree";
	node_info_text.id = node;
	node_info_text.action = visualization_msgs::Marker::ADD;
	node_info_text.pose.orientation.w = 1.0;
	node_info_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	node_info_text.scale.z = 0.5f;
	node_info_text.color.r = 1.0f;
	node_info_text.color.g = 1.0f;
	node_info_text.color.b = 1.0f;
	node_info_text.color.a = 1.0f;
	node_info_text.pose.position.x = node_position.x;
	node_info_text.pose.position.y = node_position.y;
	node_info_text.pose.position.z = node_position.z + 0.5;
	std::ostringstream oss;
	if (reward_function > 0)
		oss << "(" << node << ")" << std::fixed << std::setprecision(3)
				<< reward_function;
	else
		oss << "(" << node << ")";
	node_info_text.text = oss.str();
	_node_info_texts.markers.push_back(node_info_text);
}

std_msgs::ColorRGBA RneVisualizer::getColor(
		const rrg_nbv_exploration_msgs::Node &node) {
	std_msgs::ColorRGBA color;
	color.a = 1.0f;
	if (node.gain == -1) {
		color.r = 0.9f;
		color.g = 0.9f;
		color.b = 0.9f;
	} else {
		switch (node.status) {
		case rrg_nbv_exploration_msgs::Node::EXPLORED:
			color.g = 0.6f;
			break;
		case rrg_nbv_exploration_msgs::Node::VISITED:
			color.g = 1.0f;
			break;
		case rrg_nbv_exploration_msgs::Node::FAILED:
			color.r = 1.0f;
			break;
		case rrg_nbv_exploration_msgs::Node::ACTIVE_VISITED:
			color.r = 1.0f;
			color.g = 0.6f;
			break;
		case rrg_nbv_exploration_msgs::Node::ACTIVE:
			color.r = 1.0f;
			color.g = 1.0f;
			break;
		default:
			color.b = 1.0f;
			break;
		}
	}
	return color;
}
}
