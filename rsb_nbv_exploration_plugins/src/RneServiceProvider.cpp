/*
 * RneServiceProvider.cpp
 *
 *  Created on: Feb 17, 2020
 *      Author: marco
 */

#include <rsb_nbv_exploration_plugins/RneServiceProvider.h>

namespace rsm {

RneServiceProvider::RneServiceProvider() {
	ros::NodeHandle nh("rsm");
	_exploration_goal_subscriber = nh.subscribe("explorationGoalStatus", 1,
			&RneServiceProvider::explorationGoalCallback, this);
	_exploration_mode_subscriber = nh.subscribe("explorationMode", 1,
			&RneServiceProvider::explorationModeCallback, this);
	_state_info_subscriber = nh.subscribe("stateInfo", 10,
			&RneServiceProvider::stateInfoCallback, this);
	ros::NodeHandle rne_nh("rne");
	_update_current_goal_service = rne_nh.serviceClient<
			rsb_nbv_exploration_msgs::UpdateCurrentGoal>("updateCurrentGoal");
	_set_rrt_state_service = rne_nh.serviceClient<std_srvs::SetBool>(
			"setRneState");

	_exploration_mode = 0;
	_goal_obsolete = false;
	_exploration_running = false;
	_goal_updated = false;
	_reset_current_goal = false;
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

bool RneServiceProvider::newGoal(geometry_msgs::Pose goal) {
	if (_reset_current_goal) {
		geometry_msgs::Pose newGoal;
		_current_goal = newGoal;
		_reset_current_goal = false;
		return false;
	}
	if (goal.position.x == _current_goal.position.x
			&& goal.position.y == _current_goal.position.y
			&& goal.position.z == _current_goal.position.z
			&& goal.orientation.x == _current_goal.orientation.x
			&& goal.orientation.y == _current_goal.orientation.y
			&& goal.orientation.z == _current_goal.orientation.z
			&& goal.orientation.w == _current_goal.orientation.w) {
		return false;
	} else {
		_current_goal = goal;
		return true;
	}
}

void RneServiceProvider::explorationGoalCallback(
		const rsm_msgs::GoalStatus::ConstPtr &goal_status) {
	if (goal_status->goal_status != rsm_msgs::GoalStatus::ACTIVE
			&& newGoal(goal_status->goal)) {
		rsb_nbv_exploration_msgs::UpdateCurrentGoal srv;
		if (goal_status->goal_status == rsm_msgs::GoalStatus::REACHED) {
			srv.request.status = rsb_nbv_exploration_msgs::Node::VISITED;
		} else if (goal_status->goal_status == rsm_msgs::GoalStatus::FAILED) {
			srv.request.status = rsb_nbv_exploration_msgs::Node::FAILED;
		} else if (goal_status->goal_status == rsm_msgs::GoalStatus::ABORTED) {
			srv.request.status = rsb_nbv_exploration_msgs::Node::ABORTED;
		}
		if (!_update_current_goal_service.call(srv)) {
			ROS_ERROR("Failed to call Update Current Goal service");
		}
	}
}

void RneServiceProvider::explorationModeCallback(
		const std_msgs::Bool::ConstPtr &exploration_mode) {
	if (_exploration_mode != exploration_mode->data) {
		_exploration_mode = exploration_mode->data;
		if (_exploration_mode) {
			ros::NodeHandle rne_nh("rne");
			_exploration_goal_obsolete_subscriber = rne_nh.subscribe(
					"explorationGoalObsolete", 1,
					&RneServiceProvider::explorationGoalObsoleteCallback, this);
			ros::NodeHandle nh("rsm");
			_goal_obsolete_publisher = nh.advertise<std_msgs::Bool>(
					"goalObsolete", 1);
		} else {
			_exploration_goal_obsolete_subscriber.shutdown();
			_goal_obsolete_publisher.shutdown();
		}
	}
}

void RneServiceProvider::explorationGoalObsoleteCallback(
		const rsb_nbv_exploration_msgs::ExplorationGoalObsolete::ConstPtr &exploration_goal_obsolete) {
	if (_exploration_mode && exploration_goal_obsolete->goal_obsolete) {
//		if (!_goal_obsolete)
//			ROS_INFO_STREAM(
//					"Goal obsolete, best goal: " << exploration_goal_obsolete->best_node << " current goal: " << exploration_goal_obsolete->current_goal);
		_goal_obsolete = true;
	} else {
		_goal_obsolete = false;
	}

	if (_goal_updated && !exploration_goal_obsolete->goal_updated) {
		_reset_current_goal = true;
	}
	_goal_updated = exploration_goal_obsolete->goal_updated;
}

void RneServiceProvider::stateInfoCallback(
		const std_msgs::String::ConstPtr &state_info) {
	bool changed = false;
	if (state_info->data.rfind("E:") == 0) {
		if (!_exploration_running) {
			_exploration_running = true;
			changed = true;
		}
	} else {
		if (_exploration_running) {
			_exploration_running = false;
			changed = true;
		}
	}
	if (changed) {
		std_srvs::SetBool srv;
		srv.request.data = _exploration_running;
		if (!_set_rrt_state_service.call(srv)) {
			ROS_ERROR("Failed to call Set RRT State service");
		}
	}
}

}

