/*
 * GlobalPlanner.cpp
 *
 *  Created on: May 12, 2020
 *      Author: marco
 */

#include <rrt_nbv_exploration_plugins/RneGlobalPlanner.h>

namespace rrt_nbv_exploration {

RneGlobalPlanner::RneGlobalPlanner() {

}

RneGlobalPlanner::RneGlobalPlanner(std::string name,
		costmap_2d::Costmap2DROS* costmap_ros) {
	initialize(name, costmap_ros);
}

void RneGlobalPlanner::initialize(std::string name,
		costmap_2d::Costmap2DROS* costmap_ros) {
	ros::NodeHandle private_nh("~");
	_plan_publisher = private_nh.advertise<nav_msgs::Path>("plan", 1);
	ros::NodeHandle rne_nh("rne");
	_request_path_service = rne_nh.serviceClient<
			rrt_nbv_exploration_msgs::RequestPath>("requestPath");
}

bool RneGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& plan) {
	rrt_nbv_exploration_msgs::RequestPath srv;
	if (_request_path_service.call(srv)) {
		plan = srv.response.path;
	} else {
		ROS_ERROR("Failed to call Request Path service");
	}
	nav_msgs::Path gui_path;
	gui_path.header.frame_id = "map";
	gui_path.header.stamp = ros::Time::now();
	gui_path.poses = plan;
	_plan_publisher.publish(gui_path);
	return true;
}

}

PLUGINLIB_EXPORT_CLASS(rrt_nbv_exploration::RneGlobalPlanner,
		nav_core::BaseGlobalPlanner)
