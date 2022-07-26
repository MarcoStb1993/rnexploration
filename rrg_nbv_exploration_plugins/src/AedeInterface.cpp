/*
 * AedeInterface.cpp
 *
 *  Created on: Jul 26, 2022
 *      Author: marco
 */

#include "AedeInterface.h"

namespace rne {

AedeInterface::AedeInterface() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");
	private_nh.param<double>("position_tolerance", _position_tolerance, 0.1);

	ros::NodeHandle nh("rne");
	_request_goal_service = nh.serviceClient<
			rrg_nbv_exploration_msgs::RequestGoal>("requestGoal");
	_update_current_goal_service = nh.serviceClient<
			rrg_nbv_exploration_msgs::UpdateCurrentGoal>("updateCurrentGoal");
	_set_rrt_state_service = nh.serviceClient<std_srvs::SetBool>("setRrgState");
	_exploration_goal_obsolete_subscriber = nh.subscribe(
			"explorationGoalObsolete", 1,
			&AedeInterface::explorationGoalObsoleteCallback, this);
}

AedeInterface::~AedeInterface() {
}

void AedeInterface::updatePosition() {

}

geometry_msgs::Pose AedeInterface::getRobotPose() {
	geometry_msgs::Pose robot_pose;
	try {
		geometry_msgs::TransformStamped transformStamped =
				_tf_buffer.lookupTransform("map", _robot_frame, ros::Time(0));
		robot_pose.position.x = transformStamped.transform.translation.x;
		robot_pose.position.y = transformStamped.transform.translation.y;
		robot_pose.position.z = transformStamped.transform.translation.z;
		robot_pose.orientation = transformStamped.transform.rotation;
	} catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
	}
	return robot_pose;
}

void AedeInterface::requestNewGoal() {
	rrg_nbv_exploration_msgs::RequestGoal srv;
	if (_request_goal_service.call(srv)) {
		if (srv.response.goal_available) {
			_current_goal = srv.response.goal;
		}
	} else {
		ROS_ERROR("Failed to call Request Goal service");
	}
}

void AedeInterface::requestPath() {
	rrg_nbv_exploration_msgs::RequestPath srv;
	if (_request_path_service.call(srv)) {
		_current_plan = srv.response.path;
	} else {
		ROS_ERROR("Failed to call Request Path service");
	}
}

void AedeInterface::updateCurrentGoal() {
	rrg_nbv_exploration_msgs::UpdateCurrentGoal srv;
//	if (goal_status->goal_status == rsm_msgs::GoalStatus::REACHED) {
	srv.request.status = rrg_nbv_exploration_msgs::Node::VISITED;
//	} else if (goal_status->goal_status == rsm_msgs::GoalStatus::FAILED) {
//		srv.request.status = rrg_nbv_exploration_msgs::Node::FAILED;
//	} else if (goal_status->goal_status == rsm_msgs::GoalStatus::ABORTED) {
//		srv.request.status = rrg_nbv_exploration_msgs::Node::ABORTED;
//	}
	if (!_update_current_goal_service.call(srv)) {
		ROS_ERROR("Failed to call Update Current Goal service");
	}
}

void AedeInterface::setExplorationState(bool running) {
	std_srvs::SetBool srv;
	srv.request.data = running;
	if (!_set_rrt_state_service.call(srv)) {
		ROS_ERROR("Failed to call Set RRT State service");
	}
}

void AedeInterface::explorationGoalObsoleteCallback(
		const rrg_nbv_exploration_msgs::ExplorationGoalObsolete::ConstPtr &exploration_goal_obsolete) {
//	if (_exploration_mode && exploration_goal_obsolete->goal_obsolete) {
////		if (!_goal_obsolete)
////			ROS_INFO_STREAM(
////					"Goal obsolete, best goal: " << exploration_goal_obsolete->best_node << " current goal: " << exploration_goal_obsolete->current_goal);
//		_goal_obsolete = true;
//	} else {
//		_goal_obsolete = false;
//	}
//
//	if (_goal_updated && !exploration_goal_obsolete->goal_updated) {
//		_reset_current_goal = true;
//	}
//	_goal_updated = exploration_goal_obsolete->goal_updated;
}

}

