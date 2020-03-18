#include "rrt_nbv_exploration/RneVisualizer.h"

namespace rrt_nbv_exploration
{
RneVisualizer::RneVisualizer()
{
	ros::NodeHandle nh("rne");
    _rrt_tree_sub = nh.subscribe("rrt_tree", 1000, &RneVisualizer::visualizeRrtTree, this);
    _rrt_tree_visualization_pub = nh.advertise<visualization_msgs::Marker>("rrt_tree_vis", 1000);
    _rrt_tree_text_info_visualization_pub = nh.advertise<visualization_msgs::MarkerArray>("rrt_tree_vis_info", 1000);
}

RneVisualizer::~RneVisualizer() {

}

void RneVisualizer::initializeVisualization()  {
    _node_points.header.frame_id = "/map";
    _node_points.ns = "rrt_tree";
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
    _edge_line_list.ns = "rrt_tree";
    _edge_line_list.id = 1;
    _edge_line_list.action = visualization_msgs::Marker::ADD;
    _edge_line_list.pose.orientation.w = 1.0;
    _edge_line_list.type = visualization_msgs::Marker::LINE_LIST;
    _edge_line_list.scale.x = 0.1f;
    _edge_line_list.color.b = 1.0f;
    _edge_line_list.color.a = 1.0f;
}

void RneVisualizer::visualizeRrtTree(const rrt_nbv_exploration_msgs::Tree::ConstPtr& rrt)  {
    _node_points.header.stamp = ros::Time::now();
    _node_points.points.clear();
    _edge_line_list.points.clear();
    for (int i=0; i<rrt->node_counter; i++)
    {
        _node_points.points.push_back(rrt->nodes[i].position);
        addInfoTextVisualization(rrt->nodes[i].position, i, rrt->nodes[i].gain);
        for(int j=0; j<rrt->nodes[i].children_counter; j++)
        {
            _edge_line_list.points.push_back(rrt->nodes[i].position);
            _edge_line_list.points.push_back(rrt->nodes[rrt->nodes[i].children[j]].position);
        }
    }
    _rrt_tree_visualization_pub.publish(_node_points);
    _rrt_tree_visualization_pub.publish(_edge_line_list);
    _rrt_tree_text_info_visualization_pub.publish(_node_info_texts);
}

void RneVisualizer::addInfoTextVisualization(const geometry_msgs::Point node_position, int node, int gain)
{
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
    oss << "(" << node << ")" << std::setprecision(2) << gain;
    node_info_text.text = oss.str();
    _node_info_texts.markers.push_back(node_info_text);
}
}
