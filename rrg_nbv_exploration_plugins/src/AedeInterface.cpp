/*
 * AedeInterface.cpp
 *
 *  Created on: Jul 26, 2022
 *      Author: marco
 */

#include <rrg_nbv_exploration_plugins/AedeInterface.h>

namespace rne {

AedeInterface::AedeInterface() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");
	double position_tolerance;
	private_nh.param<double>("position_tolerance", position_tolerance, 0.1);
	_goal_tolerance_squared = pow(position_tolerance, 2);
	_waypoint_tolerance_squared = pow(2 * position_tolerance, 2);
	private_nh.param("idle_timer_duration", _idle_timer_duration, 5.0);
	_way_point_publisher = _nh.advertise<geometry_msgs::PointStamped>(
			"way_point", 1, true);
	_stop_aede_publisher = _nh.advertise<std_msgs::Int8>("stop", 1, true);

	ros::NodeHandle nh("rne");
	_request_goal_service = nh.serviceClient<
			rrg_nbv_exploration_msgs::RequestGoal>("requestGoal");
	_request_path_service = nh.serviceClient<
			rrg_nbv_exploration_msgs::RequestPath>("requestPath");
	_update_current_goal_service = nh.serviceClient<
			rrg_nbv_exploration_msgs::UpdateCurrentGoal>("updateCurrentGoal");
	_set_rrt_state_service = nh.serviceClient<std_srvs::SetBool>("setRrgState");
	_exploration_goal_obsolete_subscriber = nh.subscribe(
			"explorationGoalObsolete", 1,
			&AedeInterface::explorationGoalObsoleteCallback, this);
	_path_publisher = nh.advertise<nav_msgs::Path>("path", 1, true);
	_idle_timer = nh.createTimer(ros::Duration(_idle_timer_duration),
			&AedeInterface::idleTimerCallback, this, false, false);
	_follows_goal = false;
	_exploration_running = false;
	_idle_timer_fired_on_goal = false;
}

AedeInterface::~AedeInterface() {
}

void AedeInterface::updatePosition() {
	if (_exploration_running) {
		if (!_follows_goal) {
			requestPath();
		}
		if (_follows_goal) {
			_current_position = getRobotPose();
			if (isAtWaypoint(_current_plan.size() == 1)) {
				waypointReached();
			}
			publishWayPoint();
			publishPath();
			publishStopAede(false);
			return;
		}
	}
	publishStopAede(true);
}

void AedeInterface::waypointReached() {
	if (_current_plan.size() <= 1) { //last waypoint in plan=goal
		ROS_INFO_STREAM("Goal reached");
		updateCurrentGoal(rrg_nbv_exploration_msgs::Node::VISITED);
		_follows_goal = false;
	} else {
		_current_plan.erase(_current_plan.begin()); //remove first element from plan
		_current_waypoint = _current_plan.front().pose.position;
	}
}

void AedeInterface::startExploration() {
	_exploration_running = true;
}

void AedeInterface::stopExploration() {
	_exploration_running = false;
}

bool AedeInterface::isAtWaypoint(bool goal) {
	return squaredDistance(_current_position.position, _current_waypoint)
			<= (goal ? _goal_tolerance_squared : _waypoint_tolerance_squared);
}

void AedeInterface::checkIfRobotMoved() {
	bool robot_moved = squaredDistance(_current_position.position,
			_last_position.position) > _goal_tolerance_squared
			|| std::abs(
					getYawFromPose(_current_position)
							- getYawFromPose(_last_position))
					> _goal_tolerance_squared;
	_last_position = _current_position;
	if (!robot_moved) {
		if (!_idle_timer_fired_on_goal && _current_plan.size() > 1) {
			ROS_ERROR_STREAM("Robot did not move, try next waypoint!");
			_current_plan.erase(_current_plan.begin()); //remove first element from plan
			_current_waypoint = _current_plan.front().pose.position;
			_idle_timer_fired_on_goal = true;
		} else {
			ROS_ERROR_STREAM("Robot did not move, update goal!");
			_current_plan.clear();
			_follows_goal = false;
			updateCurrentGoal(rrg_nbv_exploration_msgs::Node::FAILED);
		}
	}
}

double AedeInterface::squaredDistance(geometry_msgs::Point p1,
		geometry_msgs::Point p2) {
	return pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2);
}

double AedeInterface::getYawFromPose(geometry_msgs::Pose &pose) {
	// get current robot orientation (yaw) for heading change calculation
	tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
			pose.orientation.z, pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

geometry_msgs::Pose AedeInterface::getRobotPose() {
	geometry_msgs::Pose robot_pose;
	try {
		geometry_msgs::TransformStamped transformStamped =
				_tf_buffer.lookupTransform("map", _robot_frame, ros::Time(0));
		robot_pose.position.x = transformStamped.transform.translation.x;
		robot_pose.position.y = transformStamped.transform.translation.y;
		robot_pose.position.z = transformStamped.transform.translation.z;
		robot_pose.orientation = transformStamped.transform.rotation;
	} catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
	}
	return robot_pose;
}

void AedeInterface::requestPath() {
	rrg_nbv_exploration_msgs::RequestGoal goal;
	if (_request_goal_service.call(goal)) {
		if (goal.response.goal_available) {
			rrg_nbv_exploration_msgs::RequestPath srv;
			if (_request_path_service.call(srv)) {
				_current_plan = srv.response.path;
				_follows_goal = true;
				_idle_timer_fired_on_goal = false;
				_current_waypoint = _current_plan.front().pose.position;
				_idle_timer.start();
				if (isAtWaypoint(false)) {
					waypointReached();
				}
				return;
			}
		}
	}
}

void AedeInterface::updateCurrentGoal(uint8_t status) {
	_idle_timer.stop();
	rrg_nbv_exploration_msgs::UpdateCurrentGoal srv;
	srv.request.status = status;
	if (!_update_current_goal_service.call(srv)) {
		ROS_ERROR("Failed to call Update Current Goal service");
	}
}

void AedeInterface::publishWayPoint() {
	geometry_msgs::PointStamped waypoint;
	waypoint.header.frame_id = "map";
	waypoint.header.stamp = ros::Time::now();
	waypoint.point = _current_waypoint;
	_way_point_publisher.publish(waypoint);
}

void AedeInterface::publishPath() {
	nav_msgs::Path path;
	path.header.frame_id = "map";
	path.header.stamp = ros::Time::now();
	path.poses = _current_plan;
	//add robot position as first pose for visualization
	geometry_msgs::PoseStamped robot_pos;
	robot_pos.pose = _current_position;
	path.poses.insert(path.poses.begin(), robot_pos);
	_path_publisher.publish(path);
}

void AedeInterface::publishStopAede(bool stop) {
	std_msgs::Int8 stop_aede;
	stop_aede.data = stop ? 10 : 0;
	_stop_aede_publisher.publish(stop_aede);
}

void AedeInterface::explorationGoalObsoleteCallback(
		const rrg_nbv_exploration_msgs::ExplorationGoalObsolete::ConstPtr &exploration_goal_obsolete) {
	if (exploration_goal_obsolete->goal_obsolete) {
		updateCurrentGoal(rrg_nbv_exploration_msgs::Node::ABORTED);
		_follows_goal = false;
		_current_plan.clear();
	}
}

void AedeInterface::idleTimerCallback(const ros::TimerEvent &event) {
	checkIfRobotMoved();
}

}

