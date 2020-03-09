#ifndef RNEXPLORATIONSTATE_H
#define RNEXPLORATIONSTATE_H

#include <pluginlib/class_list_macros.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/TeleoperationState.h>
#include <rsm_core/StateInterface.h>
#include <rsm_msgs/SetNavigationGoal.h>
#include <rrt_nbv_exploration_msgs/RequestGoal.h>
#include <tf/transform_listener.h>

namespace rsm {

/**
 * @class   RnExplorationState
 * @brief   State for choosing a goal from all nodes in the RRT. The goals are ranked by gain from ray-casting
 * 			and the best is forwarded to navigation.
 */
class RnExplorationState: public BaseState {

public:

	/**
	 * @brief Constructor
	 */
	RnExplorationState();

	/**
	 * @brief Destructor
	 */
	~RnExplorationState();

	/**
	 * @brief Called once when registered at StateInterface
	 */
	void onSetup();

	/**
	 * @brief Called once when activated
	 */
	void onEntry();

	/**
	 * @brief Process method (step-wise, never block this method)
	 */
	void onActive();

	/**
	 * @brief Called once when left
	 */
	void onExit();

	/**
	 * Called when exploration was started manually
	 */
	void onExplorationStart(bool &success, std::string &message);

	/**
	 * @brief Called when exploration was stopped manually
	 */
	void onExplorationStop(bool &success, std::string &message);

	/**
	 * Called when waypoint following was started/paused manually
	 */
	void onWaypointFollowingStart(bool &success, std::string &message);

	/**
	 * Called when waypoint following was stopped manually
	 */
	void onWaypointFollowingStop(bool &success, std::string &message);

	/**
	 * @brief Called when an operation mode interrupt was received
	 * @param interrupt Kind of interrupt (0=EmergencyStop, 1=TeleoperationInterupt)
	 */
	void onInterrupt(int interrupt);

private:

	ros::NodeHandle _nh;
	ros::ServiceClient _set_navigation_goal_service;
	ros::ServiceClient _request_goal_service;

	/**
	 * Chosen goal to be forwarded to navigation
	 */
	geometry_msgs::Pose _goal;

	/**
	 * Initiate transition to idle state
	 */
	void abortRnExplorationGoal();
};

}

#endif
