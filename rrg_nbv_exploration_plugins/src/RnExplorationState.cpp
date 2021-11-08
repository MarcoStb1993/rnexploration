#include <rrg_nbv_exploration_plugins/RnExplorationState.h>

namespace rsm {

RnExplorationState::RnExplorationState() {

}

RnExplorationState::~RnExplorationState() {
}

void RnExplorationState::onSetup() {
	//initialize services, publisher and subscriber
	ros::NodeHandle nh("rne");
	_request_goal_service = nh.serviceClient
			< rrg_nbv_exploration_msgs::RequestGoal > ("requestGoal");

	ros::NodeHandle nh_rsm("rsm");
	_set_navigation_goal_service = nh_rsm.serviceClient
			< rsm_msgs::SetNavigationGoal > ("setNavigationGoal");

	//initialize variables
	_name = "E: RN Exploration";
}

void RnExplorationState::onEntry() {

}

void RnExplorationState::onActive() {
	rrg_nbv_exploration_msgs::RequestGoal srv;
	if (_request_goal_service.call(srv)) {
		if (srv.response.exploration_finished) {
			abortRnExplorationGoal();
		} else if (srv.response.goal_available) {
			_goal.position = srv.response.goal;
			tf2::Quaternion quaternion;
			quaternion.setRPY(0, 0,
			M_PI * srv.response.best_yaw / 180.0);
			quaternion.normalize();
			_goal.orientation = tf2::toMsg(quaternion);
			if (!_interrupt_occured) {
				_stateinterface->transitionToVolatileState(
						_stateinterface->getPluginState(NAVIGATION_STATE));
			}
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

void RnExplorationState::abortRnExplorationGoal() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(rsm::RnExplorationState, rsm::BaseState)
