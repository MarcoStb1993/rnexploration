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
	_rrt_tree_sub = rne_nh.subscribe("rrt_tree", 1000,
					&RneGlobalPlanner::rrtCallback, this);
}

bool RneGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& plan) {

	plan.push_back(start);
	nav_msgs::Path gui_path;
	gui_path.header.frame_id = "map";
	gui_path.header.stamp = ros::Time::now();
	gui_path.poses.push_back(start);
	for (int i = 0; i < 20; i++) {
		geometry_msgs::PoseStamped new_goal = goal;
		tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

		new_goal.pose.position.x = -2.5 + (0.05 * i);
		new_goal.pose.position.y = -3.5 + (0.05 * i);

		new_goal.pose.orientation.x = goal_quat.x();
		new_goal.pose.orientation.y = goal_quat.y();
		new_goal.pose.orientation.z = goal_quat.z();
		new_goal.pose.orientation.w = goal_quat.w();

		plan.push_back(new_goal);
		gui_path.poses.push_back(new_goal);
	}
	plan.push_back(goal);
	gui_path.poses.push_back(goal);
	_plan_publisher.publish(gui_path);
	return true;
}

void RneGlobalPlanner::rrtCallback(const rrt_nbv_exploration_msgs::Tree::ConstPtr& rrt){

}

}

PLUGINLIB_EXPORT_CLASS(rrt_nbv_exploration::RneGlobalPlanner,
		nav_core::BaseGlobalPlanner)
