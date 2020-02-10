#ifndef RNEXPLORATIONSTATE_H
#define RNEXPLORATIONSTATE_H

#include <pluginlib/class_list_macros.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/TeleoperationState.h>
#include <rsm_core/StateInterface.h>
#include <geometry_msgs/PoseArray.h>
#include <rsm_msgs/SetNavigationGoal.h>
#include <rsm_msgs/GetFailedGoals.h>
#include <rsm_msgs/GetRobotPose.h>
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

	/**
	 * @brief Checks if a point is different from a previously failed goals including a small tolerance
	 * @param Point that is checked against previously failed goals
	 * @return Returns if the given point is different from the previously failed goals
	 */
	bool differentFromFailedGoals(geometry_msgs::Point point);

private:

	ros::NodeHandle _nh;
	ros::Subscriber _goal_sub;
	ros::ServiceClient _get_failed_goals_service;
	ros::ServiceClient _set_navigation_goal_service;
	ros::ServiceClient _request_goal_service;
	ros::ServiceClient _get_robot_pose_service;


	/**
	 * List of previously failed goals
	 */
	std::vector<geometry_msgs::Pose> _failed_goals;
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
