/*
 * GlobalPlanner.h
 *
 *  Created on: May 12, 2020
 *      Author: marco
 */

#ifndef RRG_NBV_EXPLORATION_PLUGINS_SRC_RNEGLOBALPLANNERH_
#define RRG_NBV_EXPLORATION_PLUGINS_SRC_RNEGLOBALPLANNERH_


#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <std_msgs/String.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <rsb_nbv_exploration_msgs/RequestPath.h>
#include <global_planner/planner_core.h>

namespace rsb_nbv_exploration {

class RneGlobalPlanner: public nav_core::BaseGlobalPlanner {
public:
	RneGlobalPlanner();
	RneGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& plan);
private:
	ros::NodeHandle _nh;
	ros::Publisher _plan_publisher;
	ros::Subscriber _state_info_subscriber;
	ros::ServiceClient _request_path_service;

	global_planner::GlobalPlanner  global_planner;

	bool _exploration_running;

	void stateInfoCallback(const std_msgs::String::ConstPtr& state_info);
};

} /* namespace rsb_nbv_exploration */

#endif /* RRG_NBV_EXPLORATION_PLUGINS_SRC_RNEGLOBALPLANNERH_ */
