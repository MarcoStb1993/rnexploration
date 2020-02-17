/*
 * RneServiceProvider.cpp
 *
 *  Created on: Feb 17, 2020
 *      Author: marco
 */

#include <rrt_nbv_exploration_plugins/RneServiceProvider.h>

namespace rsm {

RneServiceProvider::RneServiceProvider() {
	ros::NodeHandle nh("rsm");
	_set_goal_obsolete_service = nh.serviceClient<std_srvs::Trigger>(
			"setGoalObsolete");
	_exploration_goal_completed_service = nh.advertiseService(
			"explorationGoalCompleted",
			&RneServiceProvider::explorationGoalCompleted, this);
	ros::NodeHandle rne_nh("rne");
	_update_current_goal_service = rne_nh.serviceClient<
			rrt_nbv_exploration_msgs::UpdateCurrentGoal>("updateCurrentGoal");
}

RneServiceProvider::~RneServiceProvider() {

}

bool RneServiceProvider::explorationGoalCompleted(
		rsm_msgs::ExplorationGoalCompleted::Request &req,
		rsm_msgs::ExplorationGoalCompleted::Response &res) {
	rrt_nbv_exploration_msgs::UpdateCurrentGoal srv;
	if (req.goal_reached) {
		srv.request.status = rrt_nbv_exploration_msgs::Node::EXPLORED;
		res.message = "Node set to explored";
	} else {
		srv.request.status = rrt_nbv_exploration_msgs::Node::FAILED;
		res.message = "Node set to failed";
	}
	if(!_update_current_goal_service.call(srv)){
		ROS_ERROR("Failed to call Update Current Goal service");
	}
	res.success = 1;
	return true;
}

void RneServiceProvider::obsolete() {
	std_srvs::Trigger srv;
	if (!_set_goal_obsolete_service.call(srv)) {
		ROS_ERROR("Failed to call Set Goal Obsolete service");
	}
}

}

