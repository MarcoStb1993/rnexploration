/*
 * RneServiceProvider.h
 *
 *  Created on: Feb 17, 2020
 *      Author: marco
 */

#ifndef RRT_NBV_EXPLORATION_PLUGINS_SRC_RNESERVICEPROVIDER_H_
#define RRT_NBV_EXPLORATION_PLUGINS_SRC_RNESERVICEPROVIDER_H_

#include <ros/ros.h>

#include <rsm_msgs/ExplorationGoalCompleted.h>
#include <rrt_nbv_exploration_msgs/UpdateCurrentGoal.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include <rrt_nbv_exploration_msgs/BestAndCurrentNode.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

namespace rsm {

class RneServiceProvider {
public:
	RneServiceProvider();
	virtual ~RneServiceProvider();
	void publishTopics();

private:
	ros::NodeHandle _nh;

	ros::ServiceServer _exploration_goal_completed_service;
	ros::ServiceClient _set_goal_obsolete_service;
	ros::ServiceClient _update_current_goal_service;
	ros::Subscriber _best_and_current_goal_subscriber;
	ros::Subscriber _exploration_mode_subscriber;
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
	 * Publish if current exploration goal is obsolete if exploration mode is set to interrupt
	 */
	void publishGoalObsolete();

	bool explorationGoalCompleted(
			rsm_msgs::ExplorationGoalCompleted::Request &req,
			rsm_msgs::ExplorationGoalCompleted::Response &res);

	void bestGoalCallback(
			const rrt_nbv_exploration_msgs::BestAndCurrentNode::ConstPtr& best_goal);

	/**
	 * Callback for exploration mode
	 * @param exploration_mode Exploration mode (0=complete goal, 1=interrupt goal when exploration goals vanished)
	 */
	void explorationModeCallback(
			const std_msgs::Bool::ConstPtr& exploration_mode);
};
}
#endif /* RRT_NBV_EXPLORATION_PLUGINS_SRC_RNESERVICEPROVIDER_H_ */
