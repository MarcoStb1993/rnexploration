#include "rrt_nbv_exploration/TreePathCalculator.h"

namespace rrt_nbv_exploration {
TreePathCalculator::TreePathCalculator() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame, "base_link");
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

double TreePathCalculator::getDistanceToNode(geometry_msgs::Point node) {
	geometry_msgs::Point robot = getRobotPose().position;
	return sqrt(
			pow(robot.x - node.x, 2) + pow(robot.y - node.y, 2)
					+ pow(robot.z - node.z, 2));
}

void TreePathCalculator::calculatePath(
		std::vector<geometry_msgs::PoseStamped> &path,
		rrt_nbv_exploration_msgs::Tree rrt, int start_node, int goal_node) {
//	ROS_INFO_STREAM(
//			"calculate path from " << start_node << " to " << goal_node);
	if (start_node == goal_node) { //start and goal are the same node, just rotate on spot
		geometry_msgs::PoseStamped path_pose;
		path_pose.header.frame_id = "map";
		path_pose.header.stamp = ros::Time::now();
		path_pose.pose.position = rrt.nodes[start_node].position;
		tf2::Quaternion quaternion;
		quaternion.setRPY(0, 0,
		M_PI * rrt.nodes[start_node].best_yaw / 180.0);
		quaternion.normalize();
		path_pose.pose.orientation = tf2::toMsg(quaternion);
		path.push_back(path_pose);
	} else { //build a path from start to root and from goal to root until they meet
		bool continue_start = (start_node == 0 ? false : true), continue_goal =
				(goal_node == 0 ? false : true); //is start or goal root node?
		std::vector<int> start_path, goal_path;
		start_path.push_back(start_node);
		goal_path.push_back(goal_node);
		while (continue_start || continue_goal) {
			if (continue_start) {
				start_path.push_back(rrt.nodes[start_path.back()].parent);
//				ROS_INFO_STREAM(start_path.back() << " added to start_path");
				if (start_path.back() == 0)
					continue_start = false; //root node added
				auto result = std::find(goal_path.begin(), goal_path.end(),
						start_path.back());
				if (result != goal_path.end()) { //check if new node in start path is already in goal path
//					ROS_INFO_STREAM("Found in goal_path!");
					goal_path.erase(result, goal_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
			if (continue_goal) {
				goal_path.push_back(rrt.nodes[goal_path.back()].parent);
//				ROS_INFO_STREAM(goal_path.back() << " added to goal_path");
				if (goal_path.back() == 0)
					continue_goal = false;	//root node added
				auto result = std::find(start_path.begin(), start_path.end(),
						goal_path.back());
				if (result != start_path.end()) { //check if new node in goal path is already in start path
//					ROS_INFO_STREAM("Found in start_path!");
					start_path.erase(result, start_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
		}
		start_path.insert(start_path.end(), goal_path.rbegin(),
				goal_path.rend()); //append goal path nodes to start path
		ros::Time timestmap = ros::Time::now();
//		ROS_INFO_STREAM("Nodes in path:");
		for (auto &i : start_path) { //iterate through nodes in path and add as waypoints for path
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.frame_id = "map";
			path_pose.header.stamp = timestmap;
			path_pose.pose.position = rrt.nodes[i].position;
			tf2::Quaternion quaternion;
			double yaw;
			if (&i != &start_path.back()) //if node is not last element in list, get orientation between this node and the next
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
//			ROS_INFO_STREAM(
//					i << ": x: " << path_pose.pose.position.x << ", y: " << path_pose.pose.position.y);
//			if (&i != &start_path.back()) { //add in between node
//				geometry_msgs::PoseStamped path_pose_int;
//				path_pose_int.header.frame_id = "map";
//				path_pose_int.header.stamp = timestmap;
//				geometry_msgs::Point position;
//				position.x = (rrt.nodes[*(&i + 1)].position.x
//						+ rrt.nodes[i].position.x) / 2;
//				position.y = (rrt.nodes[*(&i + 1)].position.y
//						+ rrt.nodes[i].position.y) / 2;
//				position.z = _sensor_height;
//				path_pose_int.pose.position = position;
//				path_pose_int.pose.orientation = tf2::toMsg(quaternion);
//				path.push_back(path_pose_int);

//				path_pose_int.header.frame_id = "map";
//				path_pose_int.header.stamp = timestmap;
//				position.x = (rrt.nodes[*(&i + 1)].position.x * 3
//						+ rrt.nodes[i].position.x) / 4;
//				position.y = (rrt.nodes[*(&i + 1)].position.y * 3
//						+ rrt.nodes[i].position.y) / 4;
//				position.z = 0.2;
//				path_pose_int.pose.position = position;
//				path_pose_int.pose.orientation = tf2::toMsg(quaternion);
//				path.push_back(path_pose_int);
//
//				path_pose_int.header.frame_id = "map";
//				path_pose_int.header.stamp = timestmap;
//				position.x = (rrt.nodes[*(&i + 1)].position.x
//						+ rrt.nodes[i].position.x * 3) / 4;
//				position.y = (rrt.nodes[*(&i + 1)].position.y
//						+ rrt.nodes[i].position.y * 3) / 4;
//				position.z = 0.2;
//				path_pose_int.pose.position = position;
//				path_pose_int.pose.orientation = tf2::toMsg(quaternion);
//				path.push_back(path_pose_int);
//			}
		}
		//path.erase(path.begin()); //remove start node
	}
}

}
