/*
 * GlobalPlanner.cpp
 *
 *  Created on: May 12, 2020
 *      Author: marco
 */

#include <rsb_nbv_exploration_plugins/RneGlobalPlanner.h>

namespace rsb_nbv_exploration {

RneGlobalPlanner::RneGlobalPlanner() :
		global_planner() {

}

RneGlobalPlanner::RneGlobalPlanner(std::string name,
		costmap_2d::Costmap2DROS *costmap_ros) {
	initialize(name, costmap_ros);
}

void RneGlobalPlanner::initialize(std::string name,
		costmap_2d::Costmap2DROS *costmap_ros) {
	ros::NodeHandle private_nh("~");
	_plan_publisher = private_nh.advertise<nav_msgs::Path>("plan", 1);
	ros::NodeHandle rsm_nh("rsm");
	_state_info_subscriber = rsm_nh.subscribe("stateInfo", 10,
			&RneGlobalPlanner::stateInfoCallback, this);
	ros::NodeHandle rne_nh("rne");
	_request_path_service = rne_nh.serviceClient<
			rsb_nbv_exploration_msgs::RequestPath>("requestPath");
	_exploration_running = false;
	global_planner.initialize(name, costmap_ros);
}

bool RneGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
		const geometry_msgs::PoseStamped &goal,
		std::vector<geometry_msgs::PoseStamped> &plan) {
	if (_exploration_running) {
		rsb_nbv_exploration_msgs::RequestPath srv;
		if (_request_path_service.call(srv)) {
			plan = srv.response.path;
		} else {
			ROS_ERROR("Failed to call Request Path service");
		}
	} else {
		global_planner.makePlan(start, goal, plan);
	}
	nav_msgs::Path gui_path;
	gui_path.header.frame_id = "map";
	gui_path.header.stamp = ros::Time::now();
	gui_path.poses = plan;
	_plan_publisher.publish(gui_path);
	return true;
}

void RneGlobalPlanner::stateInfoCallback(
		const std_msgs::String::ConstPtr &state_info) {
	_exploration_running = (state_info->data.rfind("E:") == 0) ? true : false;
}

}

PLUGINLIB_EXPORT_CLASS(rsb_nbv_exploration::RneGlobalPlanner,
		nav_core::BaseGlobalPlanner)
