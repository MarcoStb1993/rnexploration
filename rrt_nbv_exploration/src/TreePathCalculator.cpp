#include "rrt_nbv_exploration/TreePathCalculator.h"

namespace rrt_nbv_exploration {
TreePathCalculator::TreePathCalculator() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame, "base_footprint");
}

geometry_msgs::Pose TreePathCalculator::getRobotPose() {
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
//	ROS_INFO_STREAM("Robot position: " << robot_pose.position.x << ", " << robot_pose.position.y << ", " << robot_pose.position.z);
	return robot_pose;
}

std::vector<int> TreePathCalculator::calculatePathToRobot(int index,
		std::vector<int> parentPathtoRobot) {
	std::vector<int> path(parentPathtoRobot);
	path.push_back(index);
	return path;
}

void TreePathCalculator::updatePathsToRobot(int prevNode, int newNode,
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &it : rrt.nodes) {
		if (it.pathToRobot.size() <= 1) { //robot currently at this node
			it.pathToRobot.insert(it.pathToRobot.begin(), newNode);
		} else {
			if (it.pathToRobot.at(1) == newNode) { //robot moving towards this node
				it.pathToRobot.erase(it.pathToRobot.begin());
			} else { //robot moving away from this node
				it.pathToRobot.insert(it.pathToRobot.begin(), newNode);
			}
		}
	}
}

void TreePathCalculator::recalculatePathsToRobot(int prevNode, int newNode,
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &it : rrt.nodes) {
		it.pathToRobot = findConnectingPath(newNode, it.index, rrt);
	}
}

void TreePathCalculator::getPath(std::vector<geometry_msgs::PoseStamped> &path,
		rrt_nbv_exploration_msgs::Tree &rrt, int goal_node) {
//	ROS_INFO_STREAM(
//			"calculate path from " << start_node << " to " << goal_node);
	if (rrt.nearest_node == goal_node) { //start and goal are the same node, just rotate on spot
		geometry_msgs::PoseStamped path_pose;
		path_pose.header.frame_id = "map";
		path_pose.header.stamp = ros::Time::now();
		path_pose.pose.position = rrt.nodes[rrt.nearest_node].position;
		tf2::Quaternion quaternion;
		quaternion.setRPY(0, 0,
		M_PI * rrt.nodes[rrt.nearest_node].best_yaw / 180.0);
		quaternion.normalize();
		path_pose.pose.orientation = tf2::toMsg(quaternion);
		path.push_back(path_pose);
	} else { //build a path from start to root and from goal to root until they meet
		ros::Time timestamp = ros::Time::now();
		for (auto &i : rrt.nodes[goal_node].pathToRobot) { //iterate through nodes in path and add as waypoints for path
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.frame_id = "map";
			path_pose.header.stamp = timestamp;
			path_pose.pose.position = rrt.nodes[i].position;
			tf2::Quaternion quaternion;
			double yaw;
			if (&i != &rrt.nodes[goal_node].pathToRobot.back()) //if node is not last element in list, get orientation between this node and the next
				yaw = atan2(
						rrt.nodes[*(&i + 1)].position.y
								- rrt.nodes[i].position.y,
						rrt.nodes[*(&i + 1)].position.x
								- rrt.nodes[i].position.x);
			else
				//last node in list, use best yaw for orientation
				yaw = M_PI * rrt.nodes[i].best_yaw / 180.0;
			quaternion.setRPY(0, 0, yaw);
			quaternion.normalize();
			path_pose.pose.orientation = tf2::toMsg(quaternion);
			path.push_back(path_pose);
		}
	}
}

bool TreePathCalculator::neighbourNodes(rrt_nbv_exploration_msgs::Tree &rrt,
		int startNode, int endNode) {
	return (rrt.nodes[startNode].parent == endNode
			|| rrt.nodes[endNode].parent == startNode);
}

std::vector<int> TreePathCalculator::findConnectingPath(int startNode,
		int goalNode, rrt_nbv_exploration_msgs::Tree &rrt) {
	std::vector<int> path;
	if (startNode != goalNode) { //build a path from start to root and from goal to root until they meet
		bool continue_start = (startNode == 0 ? false : true), continue_goal = (
				goalNode == 0 ? false : true); //is start or goal root node?
		std::vector<int> start_path, goal_path;
		start_path.push_back(startNode);
		goal_path.push_back(goalNode);
		while (continue_start || continue_goal) {
			if (continue_start) {
				start_path.push_back(rrt.nodes[start_path.back()].parent);
				//ROS_INFO_STREAM(start_path.back() << " added to start_path");
				if (start_path.back() == 0)
					continue_start = false; //root node added
				auto result = std::find(goal_path.begin(), goal_path.end(),
						start_path.back());
				if (result != goal_path.end()) { //check if new node in start path is already in goal path
					//ROS_INFO_STREAM("Found in goal_path!");
					goal_path.erase(result, goal_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
			if (continue_goal) {
				goal_path.push_back(rrt.nodes[goal_path.back()].parent);
				//ROS_INFO_STREAM(goal_path.back() << " added to goal_path");
				if (goal_path.back() == 0)
					continue_goal = false;	//root node added
				auto result = std::find(start_path.begin(), start_path.end(),
						goal_path.back());
				if (result != start_path.end()) { //check if new node in goal path is already in start path
					//ROS_INFO_STREAM("Found in start_path!");
					start_path.erase(result, start_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
		}
		start_path.insert(start_path.end(), goal_path.rbegin(),
				goal_path.rend()); //append goal path nodes to start path
		path = start_path;
	} else {
		//ROS_INFO_STREAM("Start and goal node identical");
	}
	return path;
}

}
