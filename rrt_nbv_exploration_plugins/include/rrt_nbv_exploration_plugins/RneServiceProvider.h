/*
 * RneServiceProvider.h
 *
 *  Created on: Feb 17, 2020
 *      Author: marco
 */

#ifndef RRT_NBV_EXPLORATION_PLUGINS_SRC_RNESERVICEPROVIDER_H_
#define RRT_NBV_EXPLORATION_PLUGINS_SRC_RNESERVICEPROVIDER_H_

#include <ros/ros.h>

#include <rsm_msgs/GoalStatus.h>
#include <rrt_nbv_exploration_msgs/UpdateCurrentGoal.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include <rrt_nbv_exploration_msgs/BestAndCurrentNode.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "std_srvs/SetBool.h"
#include <geometry_msgs/Pose.h>

namespace rsm {

class RneServiceProvider {
public:
	RneServiceProvider();
	virtual ~RneServiceProvider();
	void publishTopics();

private:
	ros::NodeHandle _nh;

	ros::Subscriber _exploration_goal_subscriber;
	ros::ServiceClient _set_goal_obsolete_service;
	ros::ServiceClient _update_current_goal_service;
	ros::ServiceClient _set_rrt_state_service;
	ros::Subscriber _best_and_current_goal_subscriber;
	ros::Subscriber _exploration_mode_subscriber;
	ros::Subscriber _state_info_subscriber;
	ros::Publisher _goal_obsolete_publisher;

	/**
	 * Mode of exploration (0=complete goal, 1=interrupt goal when exploration goals vanished)
	 */
	bool _exploration_mode;
	/**
	 * Is navigation goal still an exploration goal
	 */
	bool _goal_obsolete;
	/**
	 * Is exploration in started or stopped state
	 */
	bool _exploration_running;
	/**
	 * Currently active goal
	 */
	geometry_msgs::Pose _current_goal;
	/**
	 * If current goal was updated
	 */
	bool _goal_updated;
	/**
	 * If the saved current goal must be reset
	 */
	bool _reset_current_goal;

	/**
	 * Publish if current exploration goal is obsolete if exploration mode is set to interrupt
	 */
	void publishGoalObsolete();

	/**
	 * Check if the given goal is a new one
	 * @param goal Goal pose to be checked
	 * @return If the goal is new
	 */
	bool newGoal(geometry_msgs::Pose goal);

	void explorationGoalCallback(
			const rsm_msgs::GoalStatus::ConstPtr &goal_status);

	void bestGoalCallback(
			const rrt_nbv_exploration_msgs::BestAndCurrentNode::ConstPtr &best_goal);

	void stateInfoCallback(const std_msgs::String::ConstPtr &state_info);

	/**
	 * Callback for exploration mode
	 * @param exploration_mode Exploration mode (0=complete goal, 1=interrupt goal when exploration goals vanished)
	 */
	void explorationModeCallback(
			const std_msgs::Bool::ConstPtr &exploration_mode);
};
}
#endif /* RRT_NBV_EXPLORATION_PLUGINS_SRC_RNESERVICEPROVIDER_H_ */
