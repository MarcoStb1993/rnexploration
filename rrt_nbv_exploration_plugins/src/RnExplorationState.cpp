#include <rrt_nbv_exploration_plugins/RnExplorationState.h>

namespace rsm {

RnExplorationState::RnExplorationState() {

}

RnExplorationState::~RnExplorationState() {
}

void RnExplorationState::onSetup() {
	//initialize services, publisher and subscriber
	ros::NodeHandle nh("rne");
	_request_goal_service = nh.serviceClient<
			rrt_nbv_exploration_msgs::RequestGoal>("requestGoal");

	ros::NodeHandle nh_rsm("rsm");
	_get_failed_goals_service = nh_rsm.serviceClient<rsm_msgs::GetFailedGoals>(
			"getFailedGoals");
	_set_navigation_goal_service = nh_rsm.serviceClient<
			rsm_msgs::SetNavigationGoal>("setNavigationGoal");
	_get_robot_pose_service = nh_rsm.serviceClient<rsm_msgs::GetRobotPose>(
			"getRobotPose");

	//initialize variables
	_name = "RN Exploration";
}

void RnExplorationState::onEntry() {
	//Request list of failed goals from Service Provider
	rsm_msgs::GetFailedGoals srv;
	if (_get_failed_goals_service.call(srv)) {
		_failed_goals = srv.response.failedGoals.poses;
	} else {
		ROS_ERROR("Failed to call Get Failed Goals service");
	}
}

void RnExplorationState::onActive() {
	rrt_nbv_exploration_msgs::RequestGoal srv;
	if (_request_goal_service.call(srv)) {
		_goal.position = srv.response.goal;
		//TODO: regard failed goalsc
		rsm_msgs::GetRobotPose srv2;
		if (_get_robot_pose_service.call(srv2)) {
			geometry_msgs::Pose current_pose = srv2.response.pose;
			double yaw = atan2(_goal.position.y - current_pose.position.y,
					_goal.position.x - current_pose.position.x);
			_goal.orientation = tf::createQuaternionMsgFromYaw(yaw);
			if (!_interrupt_occured) {
				_stateinterface->transitionToVolatileState(
						_stateinterface->getPluginState(NAVIGATION_STATE));
			}
		} else {
			ROS_ERROR("Failed to call Get Robot Pose service");
			abortRnExplorationGoal();
		}

	} else {
		ROS_ERROR("Failed to call Request Goal service");
		abortRnExplorationGoal();
	}
}

void RnExplorationState::onExit() {
	rsm_msgs::SetNavigationGoal srv;
	srv.request.goal = _goal;
	srv.request.navigationMode = EXPLORATION;
	if (_set_navigation_goal_service.call(srv)) {
	} else {
		ROS_ERROR("Failed to call Set Navigation Goal service");
	}
}

void RnExplorationState::onExplorationStart(bool &success,
		std::string &message) {
	success = true;
	message = "Exploration running";
}

void RnExplorationState::onExplorationStop(bool &success,
		std::string &message) {
	success = true;
	message = "Exploration stopped";
	abortRnExplorationGoal();
}

void RnExplorationState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void RnExplorationState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void RnExplorationState::onInterrupt(int interrupt) {
	switch (interrupt) {
	case EMERGENCY_STOP_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
		_interrupt_occured = true;
		break;
	case TELEOPERATION_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
		_interrupt_occured = true;
		break;
	case SIMPLE_GOAL_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(NAVIGATION_STATE));
		_interrupt_occured = true;
		break;
	}
}

bool RnExplorationState::differentFromFailedGoals(geometry_msgs::Point point) {
	double tolerance = 0.05;
	for (auto iterator : _failed_goals) {
		double x_dif = abs(point.x - iterator.position.x);
		double y_dif = abs(point.y - iterator.position.y);
		if (x_dif <= tolerance && y_dif <= tolerance) {
			return false;
		}
	}
	return true;
}

void RnExplorationState::abortRnExplorationGoal() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(rsm::RnExplorationState, rsm::BaseState)
