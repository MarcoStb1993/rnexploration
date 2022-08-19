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
#include <rrg_nbv_exploration_msgs/RequestGoal.h>
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

/**
 * Interface to the Autonomous Exploration Development Environment (AEDE) which is used for
 * local planning including terrain analysis. It requests the RNE's path to the current goal
 * and uses the AEDE local planner to swiftly follow.
 */
class AedeInterface {
public:
	/**
	 * @brief Constructor that initializes all publishers, subscribers, service clients and
	 * timers as well as setting the given parameters.
	 */
	AedeInterface();
	virtual ~AedeInterface();
	/**
	 * @brief Update the local planners goal depending on the robot position of the exploration
	 * is running. Requests a new path, if none is available and selects new the next way point
	 * in the provided global path as soon as the robot reaches the current one. Also checks if
	 * the robot is stuck and publishes the plan and way point.
	 */
	void updatePosition();
	/**
	 * Starts the main functionality of this class by activating the updatePosition method
	 */
	void startExploration();
	/**
	 * Stops the main functionality of this class by deactivating the updatePosition method
	 */
	void stopExploration();

private:
	ros::NodeHandle _nh;
	ros::ServiceClient _request_path_service;
	ros::ServiceClient _request_goal_service;
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
	 * to enable faster switching to the next way point
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
	 * @brief If the AEDE local planner is running
	 */
	bool _exploration_running;
	/**
	 * @brief If the idle timer fired already for the current goal
	 */
	bool _idle_timer_fired_on_goal;
	/**
	 * @brief Number of consecutive failed path requests (which indicate something went wrong)
	 */
	int _consecutive_failed_path_requests;

	/**
	 * @brief Is called when the robot is within the position tolerance of the current way point and
	 * determines if it is the goal node, which triggers a node update and a requesting a new path in
	 * the next position update. Otherwise, the next way point the path is selected as current way point
	 */
	void waypointReached();
	/**
	 * @brief Check if the robot is within position tolerance of the current way point, this is doubled
	 * for way points in the path
	 * @param If the way point is a goal node or a way point in the path, defaults to a normal way point
	 * @return If the robot is within the position tolerance of the current way point
	 */
	bool isAtWaypoint(bool goal = false);
	/**
	 * @brief Check if the robot is within position tolerance of its last position that was stored the
	 * last time this method was called. If it still is, an idle timer is started that declares the
	 * current goal node as failed if it is executed. Otherwise, the potentially running timer is stopped
	 */
	void checkIfRobotMoved();
	/**
	 * @brief Helper function to return the squared Euclidean distance between two points
	 * @param Point one
	 * @param Point two
	 * @return Squared Euclidean distance between the two points
	 */
	double squaredDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
	/**
	 * @brief Helper function to request the current robot position using the tf2 library
	 * @return Current robot pose
	 */
	geometry_msgs::Pose getRobotPose();
	/**
	 * @brief Checks if a goal is currently available and then request the path to the current goal
	 * from RNE and if it succeeds, immediately check if the robot is already at the goal
	 */
	void requestPath();
	/**
	 * @brief Updates the current goal node's status in RNE with the given status parameter
	 * @param Status which will be assigned to the current goal node
	 */
	void updateCurrentGoal(uint8_t status);
	/**
	 * @brief Publish the current way point in the /way_point topic
	 */
	void publishWayPoint();
	/**
	 * @brief Publish the path to the current goal
	 */
	void publishPath();
	/**
	 * @brief Receives if the current goal node is obsolete and updates its status to aborted
	 */
	void explorationGoalObsoleteCallback(
			const rrg_nbv_exploration_msgs::ExplorationGoalObsolete::ConstPtr &best_goal);

	/**
	 * @brief Callback for idle timer which sets the current goal status to failed
	 * @param event
	 */
	void idleTimerCallback(const ros::TimerEvent &event);
};
}
#endif /* RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEINTERFACE_H_ */
