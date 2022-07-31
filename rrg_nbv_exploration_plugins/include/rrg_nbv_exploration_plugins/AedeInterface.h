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
#include <rrg_nbv_exploration_msgs/RequestPath.h>
#include <rrg_nbv_exploration_msgs/Node.h>
#include <rrg_nbv_exploration_msgs/ExplorationGoalObsolete.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rne {

class AedeInterface {
public:
	AedeInterface();
	virtual ~AedeInterface();
	void setExplorationState(bool running);
	void updatePosition();
	void explorationStarted();

private:
	ros::NodeHandle _nh;
	ros::ServiceClient _request_path_service;
	ros::ServiceClient _update_current_goal_service;
	ros::ServiceClient _set_rrt_state_service;
	ros::Subscriber _exploration_goal_obsolete_subscriber;
	ros::Publisher _way_point_publisher;
	ros::Publisher _path_publisher;
	ros::Publisher _stop_aede_publisher;

	ros::Timer _idle_timer;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;
	/**
	 * @brief If the robot is currently following a goal
	 */
	bool _follows_goal;
	/**
	 * @brief Currently active goal
	 */
	geometry_msgs::Point _current_waypoint;
	/**
	 * @brief Path to current goal
	 */
	std::vector<geometry_msgs::PoseStamped> _current_plan;
	/**
	 * @brief Squared tolerance in m to count as being at a goal
	 */
	double _goal_tolerance_squared;
	/**
	 * @brief Squared tolerance in m to count as being at a way point which is twice that of a goal
	 */
	double _waypoint_tolerance_squared;
	/**
	 * @brief Last position of the robot
	 */
	geometry_msgs::Point _last_position;
	/**
	 * @brief Current position of the robot
	 */
	geometry_msgs::Point _current_position;
	/**
	 * @brief Time in s that the robot can remain stationary before navigation counts as aborted because the robot is stuck
	 */
	double _idle_timer_duration;
	/**
	 * @brief If the AEDE local planner is running (=0) or not (>0)
	 */
	bool _exploration_running;

	void waypointReached();
	bool isAtWaypoint(bool goal = false);
	void checkIfRobotMoved();
	double squaredDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
	geometry_msgs::Pose getRobotPose();
	void requestNewGoal();
	void requestPath();
	void updateCurrentGoal(uint8_t status);
	void publishWayPoint();
	void publishPath();
	void publishAedeStop(bool stop);

	std::string pointToString(geometry_msgs::Point p);
	std::string pathToString(std::vector<geometry_msgs::PoseStamped> path);

	void explorationGoalObsoleteCallback(
			const rrg_nbv_exploration_msgs::ExplorationGoalObsolete::ConstPtr &best_goal);

	/**
	 * @brief Callback for idle timer
	 * @param event
	 */
	void idleTimerCallback(const ros::TimerEvent &event);
};
}
#endif /* RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEINTERFACE_H_ */
