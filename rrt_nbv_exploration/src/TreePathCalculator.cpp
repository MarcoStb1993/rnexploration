#include "rrt_nbv_exploration/TreePathCalculator.h"

namespace rrt_nbv_exploration {
TreePathCalculator::TreePathCalculator() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");
	private_nh.param("edge_length", _edge_length, 1.0);
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
	return robot_pose;
}

void TreePathCalculator::initializePathToRobot(
		rrt_nbv_exploration_msgs::Node &node,
		std::vector<int> parentPathtoRobot, double parentDistanceToRobot) {
	std::vector<int> path(parentPathtoRobot);
	path.push_back(node.index);
	node.pathToRobot = path;
	node.distanceToRobot = parentDistanceToRobot + node.distanceToParent;
}

void TreePathCalculator::updatePathsToRobot(int prevNode, int newNode,
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &it : rrt.nodes) {
		bool add = true;
		if (it.pathToRobot.size() <= 1) { //robot currently at this node
			it.pathToRobot.insert(it.pathToRobot.begin(), newNode);
		} else {
			if (it.pathToRobot.at(1) == newNode) { //robot moving towards this node
				it.pathToRobot.erase(it.pathToRobot.begin());
				add = false;
			} else { //robot moving away from this node
				it.pathToRobot.insert(it.pathToRobot.begin(), newNode);
			}
		}
		it.distanceToRobot = updatePathDistance(prevNode, newNode,
				it.distanceToRobot, add, rrt);
	}
}

void TreePathCalculator::recalculatePathsToRobot(int prevNode, int newNode,
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &it : rrt.nodes) {
		it.pathToRobot = findConnectingPath(newNode, it.index, rrt);
		it.distanceToRobot = it.distanceToRobot = calculatePathDistance(rrt,
				it.pathToRobot);
	}
}

void TreePathCalculator::getNavigationPath(
		std::vector<geometry_msgs::PoseStamped> &path,
		rrt_nbv_exploration_msgs::Tree &rrt, int goal_node,
		geometry_msgs::Point robot_pose) {
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
		//compare robot distance to second node on path with edge length between first and second node to decide if first is discarded
		std::vector<int> pathToRobot = rrt.nodes[goal_node].pathToRobot;
		if (pathToRobot.size() >= 2) {
			double distance_first_second_squared = pow(
					rrt.nodes[pathToRobot.at(0)].position.x
							- rrt.nodes[pathToRobot.at(1)].position.x, 2)
					+ pow(
							rrt.nodes[pathToRobot.at(0)].position.y
									- rrt.nodes[pathToRobot.at(1)].position.y,
							2);
			double distance_second_squared = pow(
					robot_pose.x - rrt.nodes[pathToRobot.at(1)].position.x, 2)
					+ pow(
							robot_pose.y
									- rrt.nodes[pathToRobot.at(1)].position.y,
							2);
			if (distance_first_second_squared > distance_second_squared) {
				//remove first node from path since it leads backwards on the path
				pathToRobot.erase(pathToRobot.begin());
			}
		}
		for (auto &i : pathToRobot) { //iterate through nodes in path and add as waypoints for path
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.frame_id = "map";
			path_pose.header.stamp = timestamp;
			path_pose.pose.position = rrt.nodes[i].position;
			tf2::Quaternion quaternion;
			double yaw;
			if (&i == &pathToRobot.front() && &i != &pathToRobot.back()) { //if node is first element in list, add poses between robot and node
				geometry_msgs::Pose robot_pose = getRobotPose();
				robot_pose.position.z = rrt.nodes[i].position.z;
				yaw = atan2(rrt.nodes[i].position.y - robot_pose.position.y,
						rrt.nodes[i].position.x - robot_pose.position.x);
				quaternion.setRPY(0, 0, yaw);
				quaternion.normalize();
				addInterNodes(path, robot_pose.position, rrt.nodes[i].position,
						tf2::toMsg(quaternion), yaw);
			}

			if (&i != &pathToRobot.back()) //if node is not last element in list, get orientation between this node and the next
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
			//add in between nodes
			if (&i != &pathToRobot.back()) { //add in between node
				addInterNodes(path, rrt.nodes[i].position,
						rrt.nodes[*(&i + 1)].position, tf2::toMsg(quaternion),
						yaw);
			}
		}
	}
}

void TreePathCalculator::addInterNodes(
		std::vector<geometry_msgs::PoseStamped> &path,
		geometry_msgs::Point start, geometry_msgs::Point end,
		geometry_msgs::Quaternion orientation, double yaw) {
	geometry_msgs::PoseStamped path_pose;
	path_pose.header.frame_id = "map";
	path_pose.header.stamp = ros::Time::now();
	double distance = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
	for (double i = 0.1; i < distance - 0.1; i = i + 0.1) {
		geometry_msgs::Point position;
		position.x = start.x + i * cos(yaw);
		position.y = start.y + i * sin(yaw);
		position.z = (start.z * (distance - i) + end.z * i) / distance;
		path_pose.pose.position = position;
		path_pose.pose.orientation = orientation;
		path.push_back(path_pose);
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
				if (start_path.back() == 0)
					continue_start = false; //root node added
				auto result = std::find(goal_path.begin(), goal_path.end(),
						start_path.back());
				if (result != goal_path.end()) { //check if new node in start path is already in goal path
					goal_path.erase(result, goal_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
			if (continue_goal) {
				goal_path.push_back(rrt.nodes[goal_path.back()].parent);
				if (goal_path.back() == 0)
					continue_goal = false;	//root node added
				auto result = std::find(start_path.begin(), start_path.end(),
						goal_path.back());
				if (result != start_path.end()) { //check if new node in goal path is already in start path
					start_path.erase(result, start_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
		}
		start_path.insert(start_path.end(), goal_path.rbegin(),
				goal_path.rend()); //append goal path nodes to start path
		path = start_path;
	}
	return path;
}

double TreePathCalculator::updatePathDistance(int prevNode, int newNode,
		double path_length, bool add, rrt_nbv_exploration_msgs::Tree &rrt) {
	double edge_length;
	if (_edge_length > 0) { //Fixed edge length, distances between nodes are the same
		edge_length = _edge_length;
	} else { //Variable edge length
		if (rrt.nodes[prevNode].parent == newNode) {
			edge_length = rrt.nodes[prevNode].distanceToParent;
		} else {
			edge_length = rrt.nodes[newNode].distanceToParent;
		}
	}
	return add ? (path_length + edge_length) : (path_length - edge_length);
}

double TreePathCalculator::calculatePathDistance(
		rrt_nbv_exploration_msgs::Tree &rrt, std::vector<int> path) {
	if (path.size() < 2) { // No distance to calculate
		return 0;
	}
	if (_edge_length > 0) { //Fixed edge length, distances between nodes are the same
		return ((double) path.size() - 1.0) * _edge_length;
	} else { //Variable edge length
		double distance = 0;
		for (auto &i : path) {
			if (&i != &path.back()) { //not last node in path
				if (rrt.nodes[i].parent == *(&i + 1)) { //i+1 is i's parent
					distance += rrt.nodes[i].distanceToParent;
				} else { //i is (i+1)'s parent
					distance += rrt.nodes[*(&i + 1)].distanceToParent;
				}
			}
		}
		return distance;
	}
}

}
