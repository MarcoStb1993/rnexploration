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
	_exploration_mode_subscriber = nh.subscribe("explorationMode", 1,
			&RneServiceProvider::explorationModeCallback, this);
	ros::NodeHandle rne_nh("rne");
	_update_current_goal_service = rne_nh.serviceClient<
			rrt_nbv_exploration_msgs::UpdateCurrentGoal>("updateCurrentGoal");

	_exploration_mode = 0;
	_goal_obsolete = false;
}

RneServiceProvider::~RneServiceProvider() {

}

void RneServiceProvider::publishTopics() {
	if (_exploration_mode) {
		publishGoalObsolete();
	}
}

void RneServiceProvider::publishGoalObsolete() {
	std_msgs::Bool msg;
	msg.data = _goal_obsolete;
	_goal_obsolete_publisher.publish(msg);
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
	if (!_update_current_goal_service.call(srv)) {
		ROS_ERROR("Failed to call Update Current Goal service");
	}
	res.success = 1;
	return true;
}

void RneServiceProvider::explorationModeCallback(
		const std_msgs::Bool::ConstPtr& exploration_mode) {
	if (_exploration_mode != exploration_mode->data) {
		_exploration_mode = exploration_mode->data;
		if (exploration_mode) {
			ros::NodeHandle rne_nh("rne");
			_best_and_current_goal_subscriber = rne_nh.subscribe(
					"bestAndCurrentGoal", 1,
					&RneServiceProvider::bestGoalCallback, this);
			ros::NodeHandle nh("rsm");
			_goal_obsolete_publisher = nh.advertise<std_msgs::Bool>(
					"goalObsolete", 1);
		} else {
			_best_and_current_goal_subscriber.shutdown();
			_goal_obsolete_publisher.shutdown();
		}
	}
}

void RneServiceProvider::bestGoalCallback(
		const rrt_nbv_exploration_msgs::BestAndCurrentNode::ConstPtr& best_goal) {
	if (_exploration_mode && best_goal->current_goal != best_goal->best_node) {
		_goal_obsolete = true;
	} else {
		_goal_obsolete = false;
	}
}

}

