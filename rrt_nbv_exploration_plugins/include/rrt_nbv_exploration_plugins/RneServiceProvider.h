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
#include <std_srvs/Trigger.h>

namespace rsm {

class RneServiceProvider {
public:
	RneServiceProvider();
	virtual ~RneServiceProvider();

private:
	ros::NodeHandle _nh;

	ros::ServiceServer _exploration_goal_completed_service;
	ros::ServiceClient _set_goal_obsolete_service;
	ros::ServiceClient _update_current_goal_service;

	bool explorationGoalCompleted(
			rsm_msgs::ExplorationGoalCompleted::Request &req,
			rsm_msgs::ExplorationGoalCompleted::Response &res);
	void obsolete();
};
}
#endif /* RRT_NBV_EXPLORATION_PLUGINS_SRC_RNESERVICEPROVIDER_H_ */
