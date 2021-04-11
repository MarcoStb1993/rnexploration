#include "rrt_nbv_exploration/TreeConstructor.h"

namespace rrt_nbv_exploration {
TreeConstructor::TreeConstructor() :
		_tf_listener(_tf_buffer) {
}

TreeConstructor::~TreeConstructor() {
}

void TreeConstructor::initialization(geometry_msgs::Point seed) {
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_max_range", _sensor_range, 5.0);
	_radius_search_range = pow(2 * _sensor_range, 2);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	private_nh.param("edge_length", _edge_length, 1.0);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("exploration_finished_timer_duration",
			_exploration_finished_timer_duration, 10.0);
	double distance_thresh;
	private_nh.param("distance_thresh", distance_thresh, 1.0);
	_distance_thresh_squared = pow(distance_thresh, 2);
	private_nh.param("orientation_thresh", _orientation_thresh, 120.0);
	double frontier_tolerance;
	private_nh.param("frontier_tolerance", frontier_tolerance, 0.9);
	_frontier_tolerance_squared = pow(frontier_tolerance, 2);
	private_nh.param("min_frontier_size", _min_frontier_size, 1.0);
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");

	ros::NodeHandle nh("rne");
	_frontiers_publisher = nh.advertise<rrt_nbv_exploration_msgs::Frontiers>(
			"frontiers", 1);
	_best_and_current_goal_publisher = nh.advertise<
			rrt_nbv_exploration_msgs::BestAndCurrentFrontier>(
			"bestAndCurrentFrontier", 1);
	_request_goal_service = nh.advertiseService("requestGoal",
			&TreeConstructor::requestGoal, this);
	_update_current_goal_service = nh.advertiseService("updateCurrentGoal",
			&TreeConstructor::updateCurrentGoal, this);

	_exploration_finished_timer = _nh.createTimer(
			ros::Duration(_exploration_finished_timer_duration),
			&TreeConstructor::explorationFinishedTimerCallback, this, false,
			false);

	_set_rrt_state_service = nh.advertiseService("setRrtState",
			&TreeConstructor::setRrtState, this);
	_get_rrt_state_service = nh.advertiseService("getRrtState",
			&TreeConstructor::getRrtState, this);
	_reset_rrt_state_service = nh.advertiseService("resetRrtState",
			&TreeConstructor::resetRrtState, this);

	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&TreeConstructor::convertOctomapMsgToOctree, this);

	_frontier_comparator.reset(new FrontierComparator());
	_gain_calculator.reset(new GainCalculator());
	_costmap_client.reset(new Costmap2DClient(private_nh, _nh));
	_search.reset(
			new FrontierSearch(_costmap_client->getCostmap(),
					_min_frontier_size));
	_running = false;
}

bool TreeConstructor::initRrt(const geometry_msgs::Point &seed) {
	_frontiers.header.frame_id = "/map";
	_frontiers.ns = "frontiers";
	_frontier_comparator->initialization();
	_gain_calculator->precalculateGainPolls();
	_current_goal_frontier = -1;
	_update_frontiers = false;
	return true;
}

void TreeConstructor::startRrtConstruction() {
	_frontiers.frontiers.clear();
	_frontiers.blocklist.clear();
	_frontier_comparator->clear();
	_last_robot_pos = getRobotPose();
	initRrt(_last_robot_pos.position);
	findFrontiers(_last_robot_pos.position);
	_running = true;

}

void TreeConstructor::stopRrtConstruction() {
	_running = false;
}

void TreeConstructor::runRrtConstruction() {
	_frontiers.header.stamp = ros::Time::now();
	if (_running && _map_min_bounding[0] && _map_min_bounding[1]
			&& _map_min_bounding[2]) {
		determineIfRobotMoved();
		if (_update_frontiers || _current_goal_frontier == -1) {
			_update_frontiers = false;
			findFrontiers(_last_robot_pos.position);
		}
		_frontier_comparator->maintainList(_frontiers);
		checkCurrentGoal();
		publishFrontierWithBestGain();
		if (_current_goal_frontier == -1) {
			_exploration_finished_timer.start();
		}
	}
	_frontiers_publisher.publish(_frontiers);
}

void TreeConstructor::findFrontiers(const geometry_msgs::Point &center) {
	auto old_frontiers = _frontiers.frontiers;
	if (_current_goal_frontier != -1) {
		rrt_nbv_exploration_msgs::Frontier goal_frontier =
				_frontiers.frontiers[_current_goal_frontier];
		goal_frontier.index = 0;
		_frontier_comparator->addFrontier(goal_frontier.index);
		_frontiers.frontier_counter = 1;
		_frontiers.frontiers.clear();
		_frontiers.frontiers.push_back(goal_frontier);
		_current_goal_frontier = 0;
	} else {
		_frontiers.frontier_counter = 0;
		_frontiers.frontiers.clear();
	}
	auto frontiers = _search->searchFrom(_last_robot_pos.position);
	for (auto frontier : frontiers) {
		if (!isFrontierBlocklisted(frontier)) {
			if (!(_current_goal_frontier != -1
					&& isFrontierCurrentGoal(frontier, _frontiers.frontiers[0]))) {
				frontier.index = _frontiers.frontier_counter++;
				int existing_index = isFrontierPresent(frontier, old_frontiers);
				if (existing_index >= 0
						&& isFrontierOutOfSensorRange(frontier)) {
					frontier.gain = old_frontiers[existing_index].gain;
					frontier.best_yaw = old_frontiers[existing_index].best_yaw;
				} else
					_gain_calculator->calculateGain(frontier);
				_frontiers.frontiers.push_back(frontier);
				if (frontier.status
						!= rrt_nbv_exploration_msgs::Frontier::EXPLORED)
					_frontier_comparator->addFrontier(frontier.index);
			}
		}
	}
}

bool TreeConstructor::isFrontierBlocklisted(
		rrt_nbv_exploration_msgs::Frontier new_frontier) {
	for (auto blocked : _frontiers.blocklist) {
		double distance_squared = pow(new_frontier.centroid.x - blocked.x, 2)
				+ pow(new_frontier.centroid.y - blocked.y, 2);
		if (distance_squared <= _frontier_tolerance_squared) {
			return true;
		}
	}
	return false;
}

bool TreeConstructor::isFrontierCurrentGoal(
		rrt_nbv_exploration_msgs::Frontier new_frontier,
		rrt_nbv_exploration_msgs::Frontier current_goal) {
	double distance_squared = pow(
			new_frontier.centroid.x - current_goal.centroid.x, 2)
			+ pow(new_frontier.centroid.y - current_goal.centroid.y, 2);
	if (distance_squared <= _frontier_tolerance_squared)
		return true;
	else
		return false;
}

int TreeConstructor::isFrontierPresent(
		rrt_nbv_exploration_msgs::Frontier new_frontier,
		std::vector<rrt_nbv_exploration_msgs::Frontier> old_frontiers) {
	for (auto old_frontier : old_frontiers) {
		double distance_squared = pow(
				new_frontier.centroid.x - old_frontier.centroid.x, 2)
				+ pow(new_frontier.centroid.y - old_frontier.centroid.y, 2);
		if (distance_squared <= _frontier_tolerance_squared)
			return old_frontier.index;
	}
	return -1;
}

bool TreeConstructor::isFrontierOutOfSensorRange(
		rrt_nbv_exploration_msgs::Frontier new_frontier) {
	double distance_squared = pow(
			new_frontier.centroid.x - _last_robot_pos.position.x, 2)
			+ pow(new_frontier.centroid.y - _last_robot_pos.position.y, 2);
	return distance_squared > _radius_search_range;
}

void TreeConstructor::publishFrontierWithBestGain() {
	rrt_nbv_exploration_msgs::BestAndCurrentFrontier msg;
	msg.current_goal = _current_goal_frontier;
	msg.best_frontier =
			_frontier_comparator->isEmpty() ?
					_current_goal_frontier :
					_frontier_comparator->getBestFrontier();
	msg.goal_updated = _goal_updated;
	_best_and_current_goal_publisher.publish(msg);
}

void TreeConstructor::checkCurrentGoal() {
	if (_current_goal_frontier == -1 && !_frontier_comparator->isEmpty()) {
		_current_goal_frontier = _frontier_comparator->getBestFrontier();
		_moved_to_current_goal = false;
		_goal_updated = true;
		_updating = false;
		_exploration_finished_timer.stop();
		ROS_INFO_STREAM("Current goal node set to " << _current_goal_frontier);
	}
}

void TreeConstructor::determineIfRobotMoved() {
	geometry_msgs::Pose pos = getRobotPose();
	double distance_squared = pow(pos.position.x - _last_robot_pos.position.x,
			2) + pow(pos.position.y - _last_robot_pos.position.y, 2);
	tf2::Quaternion pos_quat, last_pos_quat;
	tf2::fromMsg(pos.orientation, pos_quat);
	tf2::fromMsg(_last_robot_pos.orientation, last_pos_quat);
	double pos_roll, pos_pitch, pos_yaw;
	tf2::Matrix3x3(pos_quat).getRPY(pos_roll, pos_pitch, pos_yaw);
	double last_pos_roll, last_pos_pitch, last_pos_yaw;
	tf2::Matrix3x3(last_pos_quat).getRPY(last_pos_roll, last_pos_pitch,
			last_pos_yaw);
	if (distance_squared >= _distance_thresh_squared
			|| abs(pos_yaw - last_pos_yaw) >= _orientation_thresh) {
		_last_robot_pos = pos;
		_update_frontiers = true;
	}
}

void TreeConstructor::updateCurrentGoal() {
	switch (_frontiers.frontiers[_current_goal_frontier].status) {
	case rrt_nbv_exploration_msgs::Frontier::EXPLORED:
		ROS_INFO("RNE goal explored");
		break;
	case rrt_nbv_exploration_msgs::Frontier::VISITED:
		ROS_INFO("RNE goal visited");
		break;
	case rrt_nbv_exploration_msgs::Frontier::ABORTED:
		ROS_INFO("RNE goal aborted");
		break;
	case rrt_nbv_exploration_msgs::Frontier::FAILED:
		ROS_INFO("RNE goal failed, add to blocklist");
		_frontiers.blocklist.push_back(
				_frontiers.frontiers[_current_goal_frontier].centroid);
		break;
	default:
		//active or waiting (should not occur)
		ROS_INFO("RNE goal active or waiting");
		return;
	}
	_current_goal_frontier = -1;
	_frontier_comparator->clear();
}

geometry_msgs::Pose TreeConstructor::getRobotPose() {
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

void TreeConstructor::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr &map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
	updateMapDimensions();
}

void TreeConstructor::updateMapDimensions() {
	_octree->getMetricMin(_map_min_bounding[0], _map_min_bounding[1],
			_map_min_bounding[2]);
	_octree->getMetricMax(_map_max_bounding[0], _map_max_bounding[1],
			_map_max_bounding[2]);
}

void TreeConstructor::explorationFinishedTimerCallback(
		const ros::TimerEvent &event) {
	ROS_INFO_STREAM("Exploration finished");
	_running = false;
	_exploration_finished_timer.stop();
}

bool TreeConstructor::requestGoal(
		rrt_nbv_exploration_msgs::RequestGoal::Request &req,
		rrt_nbv_exploration_msgs::RequestGoal::Response &res) {
	res.goal_available = _goal_updated && _current_goal_frontier != -1;
	res.exploration_finished = !_running;
	if (res.goal_available) {
		if (_frontiers.frontiers[_current_goal_frontier].status
				== rrt_nbv_exploration_msgs::Frontier::VISITED) {
			_frontiers.frontiers[_current_goal_frontier].status =
					rrt_nbv_exploration_msgs::Frontier::ACTIVE_VISITED;
		} else if (_frontiers.frontiers[_current_goal_frontier].status
				== rrt_nbv_exploration_msgs::Frontier::INITIAL) {
			_frontiers.frontiers[_current_goal_frontier].status =
					rrt_nbv_exploration_msgs::Frontier::ACTIVE;
		}
		res.goal = _frontiers.frontiers[_current_goal_frontier].centroid;
		res.best_yaw = _frontiers.frontiers[_current_goal_frontier].best_yaw;
		_goal_updated = false;
	}
	return true;
}

bool TreeConstructor::updateCurrentGoal(
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Response &res) {
	if (_current_goal_frontier != -1 && !_updating) {
		_updating = true;
		_frontiers.frontiers[_current_goal_frontier].status = req.status;
		updateCurrentGoal();
	}
	res.success = true;
	res.message = "Changed current goal's status";
	return true;
}

bool TreeConstructor::setRrtState(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (req.data) {
		if (_running) {
			res.success = false;
			res.message = "Tree construction already running";
		} else {
			startRrtConstruction();
			res.success = true;
			res.message = "Tree construction started";
		}
	} else {
		if (!_running) {
			res.success = false;
			res.message = "Tree construction already stopped";
		} else {
			stopRrtConstruction();
			res.success = true;
			res.message = "Tree construction stopped";
		}
	}
	return true;
}

bool TreeConstructor::getRrtState(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	res.success = _running;
	res.message = "Tree construction status sent";
	return true;
}

bool TreeConstructor::resetRrtState(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	_running = false;
	_frontiers.frontiers.clear();
	_frontiers.blocklist.clear();
	_frontier_comparator->clear();
	initRrt(geometry_msgs::Point());
	res.message = true;
	res.message = "Tree reset";
	return true;
}

}
