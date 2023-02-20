#include <rsb_nbv_exploration/RneVisualizer.h>

namespace rsb_nbv_exploration {
RneVisualizer::RneVisualizer() {
	ros::NodeHandle private_nh("~");
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("grid_map_occupied", _grid_map_occupied, 100);
	private_nh.param("show_gain_info", _show_gain_info, false);
	private_nh.param("show_distance_info", _show_distance_info, false);
	private_nh.param("show_traversability_info", _show_traversability_info,
			false);
	private_nh.param("show_heading_info", _show_heading_info, false);
	private_nh.param("show_radius_info", _show_radius_info, false);
	private_nh.param("show_cost_info", _show_cost_info, false);

	ros::NodeHandle nh("rne");
	_rrg_sub = nh.subscribe("rrg", 10, &RneVisualizer::visualizeRrgGraph, this);
	_gg_sub = nh.subscribe("globalgraph", 10, &RneVisualizer::visualizeGgGraph,
			this);
	_rrg_visualization_pub = nh.advertise<visualization_msgs::Marker>("rrg_vis",
			1000);
	_rrg_text_info_visualization_pub = nh.advertise<
			visualization_msgs::MarkerArray>("rrg_vis_info", 10);
	_gg_visualization_pub = nh.advertise<visualization_msgs::Marker>(
			"globalgraph_vis", 10);
	_gg_text_info_visualization_pub = nh.advertise<
			visualization_msgs::MarkerArray>("globalgraph_vis_info", 10);
	_info_interval = 1.0;
	_last_rrg_node_count = 0;
	_last_gg_target_count = 0;
}

RneVisualizer::~RneVisualizer() {

}

void RneVisualizer::initializeRrgVisualization(
		visualization_msgs::Marker &_node_points,
		visualization_msgs::Marker &_edge_line_list) {
	_node_points.header.frame_id = "map";
	_node_points.ns = "rrg_vis";
	_node_points.id = 0;
	_node_points.action = visualization_msgs::Marker::ADD;
	_node_points.pose.orientation.w = 1.0;
	_node_points.type = visualization_msgs::Marker::SPHERE_LIST;
	_node_points.scale.x = _robot_radius * 0.2f;
	_node_points.scale.y = _robot_radius * 0.2f;
	_node_points.scale.z = _robot_radius * 0.2f;
	_node_points.color.g = 1.0f;
	_node_points.color.a = 1.0f;
	_edge_line_list.header.frame_id = "map";
	_edge_line_list.header.stamp = ros::Time::now();
	_edge_line_list.ns = "rrg_vis";
	_edge_line_list.id = 1;
	_edge_line_list.action = visualization_msgs::Marker::ADD;
	_edge_line_list.pose.orientation.w = 1.0;
	_edge_line_list.type = visualization_msgs::Marker::LINE_LIST;
	_edge_line_list.scale.x = _robot_radius * 0.075f;
	_edge_line_list.color.b = 1.0f;
	_edge_line_list.color.a = 1.0f;
}

void RneVisualizer::visualizeRrgGraph(
		const rsb_nbv_exploration_msgs::Graph::ConstPtr &rrg) {
	if (_rrg_visualization_pub.getNumSubscribers() > 0) {
		bool clear_info_texts = false;
		bool publishInfo = _rrg_text_info_visualization_pub.getNumSubscribers()
				> 0
				&& (ros::Time::now() - _last_info_publish).toSec()
						>= _info_interval;
		visualization_msgs::Marker node_points;
		visualization_msgs::Marker edge_line_list;
		initializeRrgVisualization(node_points, edge_line_list);
		visualization_msgs::MarkerArray node_info_texts;
		node_points.header.stamp = ros::Time::now();
		node_points.points.clear();
		node_points.colors.clear();
		edge_line_list.points.clear();
		edge_line_list.colors.clear();
		for (int i = 0; i < rrg->node_counter; i++) {
			if (rrg->nodes[i].status
					!= rsb_nbv_exploration_msgs::Node::INACTIVE) {
				node_points.points.push_back(rrg->nodes[i].position);
				node_points.colors.push_back(getColor(rrg->nodes[i]));
				if (publishInfo)
					addInfoTextVisualization(node_info_texts, i, rrg);
			} else { //remove info text from inactive nodes
				clear_info_texts = true;
			}
		}
		for (int j = 0; j < rrg->edges.size(); j++) {
			if (!rrg->edges[j].inactive) {
				edge_line_list.points.push_back(
						rrg->nodes[rrg->edges[j].first_node].position);
				edge_line_list.colors.push_back(
						getColor(rrg->nodes[rrg->edges[j].first_node]));
				edge_line_list.points.push_back(
						rrg->nodes[rrg->edges[j].second_node].position);
				edge_line_list.colors.push_back(
						getColor(rrg->nodes[rrg->edges[j].second_node]));
			}
		}
		_rrg_visualization_pub.publish(node_points);
		_rrg_visualization_pub.publish(edge_line_list);
		if (publishInfo) {
			if (clear_info_texts || rrg->node_counter < _last_rrg_node_count) { //graph normally only grows
				clearInfoText();
			}
			_rrg_text_info_visualization_pub.publish(node_info_texts);
			_last_info_publish = ros::Time::now();
		}
	}
	_last_rrg_node_count = rrg->node_counter;
}

void RneVisualizer::addInfoTextVisualization(
		visualization_msgs::MarkerArray &node_info_texts, int node,
		const rsb_nbv_exploration_msgs::Graph::ConstPtr &rrg) {
	visualization_msgs::Marker node_info_text;
	node_info_text.header.frame_id = "map";
	node_info_text.ns = "rrg_vis_info";
	node_info_text.id = node;
	node_info_text.action = visualization_msgs::Marker::ADD;
	node_info_text.pose.orientation.w = 1.0;
	node_info_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	node_info_text.scale.z = 0.5f;
	node_info_text.color.r = 1.0f;
	node_info_text.color.g = 1.0f;
	node_info_text.color.b = 1.0f;
	node_info_text.color.a = 1.0f;
	node_info_text.pose.position.x = rrg->nodes[node].position.x;
	node_info_text.pose.position.y = rrg->nodes[node].position.y;
	node_info_text.pose.position.z = rrg->nodes[node].position.z + 0.5;
	std::ostringstream oss;
	if (rrg->nodes[node].reward_function > 0) {
		oss << std::setw(node < 100 ? 3 : 2) << "(" << node << ")" << "\n"
				<< std::fixed << std::setprecision(6)
				<< rrg->nodes[node].reward_function << std::fixed
				<< std::setprecision(3);
		if (_show_gain_info)
			oss << std::fixed << std::setprecision(3) << "\ng: "
					<< rrg->nodes[node].gain;
	} else {
		oss << "(" << node << ")";
	}
	if (_show_cost_info)
		oss << std::fixed << std::setprecision(3) << "\nc: "
				<< rrg->nodes[node].cost_function;
	if (_show_distance_info)
		oss << "\nd: " << rrg->nodes[node].distance_to_robot;
	if (_show_traversability_info)
		oss << "\nt: " << rrg->nodes[node].traversability_cost_to_robot;
	if (_show_heading_info)
		oss << "\nh: " << rrg->nodes[node].heading_change_to_robot_best_view;
	if (_show_radius_info)
		oss << "\nr: "
				<< (rrg->nodes[node].radii_to_robot
						/ rrg->nodes[node].path_to_robot.size() / _robot_radius);
	node_info_text.text = oss.str();
	node_info_texts.markers.push_back(node_info_text);
}

void RneVisualizer::clearInfoText() {
	visualization_msgs::Marker node_info_text;
	node_info_text.header.frame_id = "map";
	node_info_text.ns = "rrg_vis_info";
	node_info_text.id = 0;
	node_info_text.action = visualization_msgs::Marker::DELETEALL;
	visualization_msgs::MarkerArray _node_info_texts;
	_node_info_texts.markers.push_back(node_info_text);
	_rrg_text_info_visualization_pub.publish(_node_info_texts);
}

std_msgs::ColorRGBA RneVisualizer::getColor(
		const rsb_nbv_exploration_msgs::Node &node) {
	std_msgs::ColorRGBA color;
	color.a = 1.0f;
	if (node.gain == -1) {
		color.r = 0.9f;
		color.g = 0.9f;
		color.b = 0.9f;
	} else {
		switch (node.status) {
		case rsb_nbv_exploration_msgs::Node::EXPLORED:
			color.g = 0.6f;
			break;
		case rsb_nbv_exploration_msgs::Node::VISITED:
			color.g = 1.0f;
			break;
		case rsb_nbv_exploration_msgs::Node::FAILED:
			color.r = 1.0f;
			break;
		case rsb_nbv_exploration_msgs::Node::ACTIVE_VISITED:
			color.r = 1.0f;
			color.g = 0.6f;
			break;
		case rsb_nbv_exploration_msgs::Node::ACTIVE:
			color.r = 1.0f;
			color.g = 1.0f;
			break;
		default:
			color.b = 1.0f;
			color.g = 1 - node.reward_function;
			break;
		}
	}
	return color;
}

void RneVisualizer::initializeGgVisualization(
		visualization_msgs::Marker &target_points,
		visualization_msgs::Marker &path_lines) {
	target_points.header.frame_id = "map";
	target_points.ns = "globalgraph_vis";
	target_points.id = 0;
	target_points.action = visualization_msgs::Marker::ADD;
	target_points.pose.orientation.w = 1.0;
	target_points.type = visualization_msgs::Marker::CUBE_LIST;
	target_points.scale.x = _robot_radius * 0.2f;
	target_points.scale.y = _robot_radius * 0.2f;
	target_points.scale.z = _robot_radius * 0.2f;
	target_points.color.g = 1.0f;
	target_points.color.a = 1.0f;
	path_lines.header.frame_id = "map";
	path_lines.ns = "globalgraph_vis";
	path_lines.id = 1;
	path_lines.action = visualization_msgs::Marker::ADD;
	path_lines.pose.orientation.w = 1.0;
	path_lines.type = visualization_msgs::Marker::LINE_LIST;
	path_lines.scale.x = _robot_radius * 0.125f;
	path_lines.color.g = 1.0f;
	path_lines.color.a = 1.0f;
}

void RneVisualizer::visualizeGgGraph(
		const rsb_nbv_exploration_msgs::GlobalGraph::ConstPtr &gg) {
	if (_gg_visualization_pub.getNumSubscribers() > 0) {
		bool clear_info_texts = false;
		bool clear_global_connections = false;
		bool publishInfo = _gg_text_info_visualization_pub.getNumSubscribers()
				> 0
				&& (ros::Time::now() - _last_gg_info_publish).toSec()
						>= _info_interval;
		visualization_msgs::Marker target_points;
		visualization_msgs::Marker path_lines;
		visualization_msgs::MarkerArray target_info_texts;
		initializeGgVisualization(target_points, path_lines);
		for (int i = 0; i < gg->targets_counter; i++) {
			if (!gg->targets[i].inactive) {
				target_points.points.push_back(gg->targets[i].viewpoint);
				target_points.colors.push_back(
						getFrontierColor(gg->targets[i].index));
				if (publishInfo)
					addGgInfoTextVisualization(target_info_texts, i, gg);
			} else
				clear_info_texts = true;
		}
		for (int i = 0; i < gg->connections_counter; i++) {
			if (!gg->connections[i].inactive) {
				for (auto j = 0; j < gg->connections[i].waypoints.size() - 1; j++) {
					path_lines.points.push_back(gg->connections[i].waypoints[j]);
					path_lines.colors.push_back(
							getFrontierColor(gg->connections[i].first_target));
					path_lines.points.push_back(gg->connections[i].waypoints[j + 1]);
					path_lines.colors.push_back(
							getFrontierColor(gg->connections[i].first_target));
				}
			} else {
				clear_global_connections = true;
			}
		}
		if (clear_global_connections)
			clearGlobalPaths();
		_gg_visualization_pub.publish(target_points);
		_gg_visualization_pub.publish(path_lines);
		if (publishInfo) {
			if (clear_info_texts
					|| gg->targets_counter < _last_gg_target_count) { //graph normally only grows
				clearGgInfoText();
			}
			_gg_text_info_visualization_pub.publish(target_info_texts);
			_last_gg_info_publish = ros::Time::now();
		}
	}
}

void RneVisualizer::addGgInfoTextVisualization(
		visualization_msgs::MarkerArray &target_info_texts, int target,
		const rsb_nbv_exploration_msgs::GlobalGraph::ConstPtr &gg) {
	visualization_msgs::Marker target_info_text;
	target_info_text.header.frame_id = "map";
	target_info_text.ns = "globalgraph_vis_info";
	target_info_text.id = target;
	target_info_text.action = visualization_msgs::Marker::ADD;
	target_info_text.pose.orientation.w = 1.0;
	target_info_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	target_info_text.scale.z = 0.5f;
	target_info_text.color.r = 1.0f;
	target_info_text.color.g = 1.0f;
	target_info_text.color.b = 1.0f;
	target_info_text.color.a = 1.0f;
	target_info_text.pose.position.x = gg->targets[target].viewpoint.x;
	target_info_text.pose.position.y = gg->targets[target].viewpoint.y;
	target_info_text.pose.position.z = gg->targets[target].viewpoint.z
			+ 0.5;
	std::ostringstream oss;
	if (gg->targets[target].merged_distance > 0)
		oss << "{" << target << "}";
	else
		oss << "[" << target << "]";
	target_info_text.text = oss.str();
	target_info_texts.markers.push_back(target_info_text);
}

void RneVisualizer::clearGgInfoText() {
	visualization_msgs::Marker target_info_text;
	target_info_text.header.frame_id = "map";
	target_info_text.ns = "globalgraph_vis_info";
	target_info_text.id = 0;
	target_info_text.action = visualization_msgs::Marker::DELETEALL;
	visualization_msgs::MarkerArray target_info_texts;
	target_info_texts.markers.push_back(target_info_text);
	_gg_text_info_visualization_pub.publish(target_info_texts);
}

void RneVisualizer::clearGlobalPaths() {
	visualization_msgs::Marker path_lines;
	path_lines.header.frame_id = "map";
	path_lines.ns = "globalgraph_vis";
	path_lines.id = 1;
	path_lines.action = visualization_msgs::Marker::DELETEALL;
	_gg_visualization_pub.publish(path_lines);
}

std_msgs::ColorRGBA RneVisualizer::getFrontierColor(int target) {
	std_msgs::ColorRGBA color;
	color.r = 0.0f;
	color.g = 0.0f;
	color.b = 0.0f;
	color.a = 1.0f;
	if (target == 0) { //origin
		color.r = 1.0f;
		color.g = 0.6f;
	} else if (target == -1) { //local graph
		color.r = 1.0f;
		color.g = 1.0f;
	} else {
		target = target % 10;
		switch (target) {
		case 0: //red
			color.r = 0.898f;
			color.g = 0.098f;
			color.b = 0.293f;
			break;
		case 1: //magenta
			color.r = 0.938f;
			color.g = 0.195f;
			color.b = 0.898f;
			break;
		case 2: //dark yellow
			color.r = 0.5f;
			color.g = 0.5f;
			color.b = 0.0f;
			break;
		case 3: //blue
			color.r = 0.0f;
			color.g = 0.508f;
			color.b = 0.098f;
			break;
		case 4: //orange
			color.r = 0.957f;
			color.g = 0.508f;
			color.b = 0.188f;
			break;
		case 5: //purple
			color.r = 0.566f;
			color.g = 0.117f;
			color.b = 0.703f;
			break;
		case 6: //cyan
			color.r = 0.273f;
			color.g = 0.938f;
			color.b = 0.938f;
			break;
		case 7: //green
			color.r = 0.234f;
			color.g = 0.703f;
			color.b = 0.293f;
			break;
		case 8: //teal
			color.r = 0.0f;
			color.g = 0.5f;
			color.b = 0.5f;
			break;
		case 9: //maroon
			color.r = 0.5f;
			color.g = 0.0f;
			color.b = 0.0f;
			break;
		default: //black
			break;
		}
	}
	return color;
}

void RneVisualizer::dynamicReconfigureCallback(
		rsb_nbv_exploration::RneVisualizerConfig &config, uint32_t level) {
	_show_gain_info = config.show_gain_info;
	_show_distance_info = config.show_distance_info;
	_show_heading_info = config.show_heading_info;
	_show_traversability_info = config.show_traversability_info;
	_show_radius_info = config.show_radius_info;
	_show_cost_info = config.show_cost_info;
}

}
