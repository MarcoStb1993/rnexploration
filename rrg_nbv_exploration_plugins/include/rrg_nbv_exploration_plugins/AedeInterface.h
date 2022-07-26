/*
 * AedeInterface.h
 *
 *  Created on: Jul 26, 2022
 *      Author: marco
 */

#ifndef RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEINTERFACE_H_
#define RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEINTERFACE_H_

#include <ros/ros.h>

#include <rsm_msgs/GoalStatus.h>
#include <rrg_nbv_exploration_msgs/UpdateCurrentGoal.h>
#include <rrg_nbv_exploration_msgs/RequestGoal.h>
#include <rrg_nbv_exploration_msgs/RequestPath.h>
#include <rrg_nbv_exploration_msgs/Node.h>
#include <rrg_nbv_exploration_msgs/ExplorationGoalObsolete.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rne {

class AedeInterface {
public:
	AedeInterface();
	virtual ~AedeInterface();
	void updatePosition();

private:
	ros::NodeHandle _nh;
	ros::ServiceClient _request_path_service;
	ros::ServiceClient _request_goal_service;
	ros::ServiceClient _update_current_goal_service;
	ros::ServiceClient _set_rrt_state_service;
	ros::Subscriber _exploration_goal_obsolete_subscriber;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;
	/**
	 * @brief Currently active goal
	 */
	geometry_msgs::Point _current_goal;
	/**
	 * @brief Path to current goal
	 */
	std::vector<geometry_msgs::PoseStamped> _current_plan;
	/**
	 * @brief Tolerance in m to count as being at a position
	 */
	double _position_tolerance;

	geometry_msgs::Pose getRobotPose();
	void requestNewGoal();
	void requestPath();
	void updateCurrentGoal();
	void setExplorationState(bool running);

	void explorationGoalObsoleteCallback(
			const rrg_nbv_exploration_msgs::ExplorationGoalObsolete::ConstPtr &best_goal);
};
}
#endif /* RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEINTERFACE_H_ */
