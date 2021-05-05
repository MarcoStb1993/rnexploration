#include <rrt_nbv_exploration/GraphPathCalculator.h>

namespace rrt_nbv_exploration {
GraphPathCalculator::GraphPathCalculator() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");
}

geometry_msgs::Pose GraphPathCalculator::getRobotPose() {
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

void GraphPathCalculator::updatePathsToRobot(int startNode,
		rrt_nbv_exploration_msgs::Graph &rrg, bool reset) {
	//run Dijkstra on RRG and assign distance and path to each node
//	ROS_INFO_STREAM(
//			"update paths to robot from start node " << startNode << (reset ? " with reset": " locally"));
	std::set<std::pair<double, int>> node_queue;
	if (reset) {
		for (auto &node : rrg.nodes) {
			node.distanceToRobot = std::numeric_limits<double>::infinity();
			node.pathToRobot.clear();
		}
		rrg.nodes[startNode].distanceToRobot = 0;
		rrg.nodes[startNode].pathToRobot.push_back(startNode);
	}
	node_queue.insert(
			std::make_pair(rrg.nodes[startNode].distanceToRobot, startNode));
	while (!node_queue.empty()) {
		double current_distance = node_queue.begin()->first;
		int current_node = node_queue.begin()->second;
		node_queue.erase(node_queue.begin());
//			ROS_INFO_STREAM(
//					"Looking at node " << current_node << " with distance to robot " << current_distance);
		for (auto edge : rrg.nodes[current_node].edges) {
			int neighbor_node =
					rrg.edges[edge].first_node == current_node ?
							rrg.edges[edge].second_node :
							rrg.edges[edge].first_node;
			double neighbor_distance = rrg.edges[edge].length;
			double total_distance = current_distance + neighbor_distance;
//				ROS_INFO_STREAM(
//						"Neighbor " << neighbor_node << " with edge length " << neighbor_distance << " has total distance " << total_distance << " compared to current: " << rrg.nodes[neighbor_node].distanceToRobot << (total_distance < rrg.nodes[neighbor_node].distanceToRobot ? " shorter": " longer"));
			if (total_distance < rrg.nodes[neighbor_node].distanceToRobot) {
				node_queue.erase(
						std::make_pair(rrg.nodes[neighbor_node].distanceToRobot,
								neighbor_node));
				rrg.nodes[neighbor_node].distanceToRobot = total_distance;
				rrg.nodes[neighbor_node].pathToRobot =
						rrg.nodes[current_node].pathToRobot;
				rrg.nodes[neighbor_node].pathToRobot.push_back(neighbor_node);
				node_queue.insert(
						std::make_pair(rrg.nodes[neighbor_node].distanceToRobot,
								neighbor_node));
//					ROS_INFO_STREAM("set node " << neighbor_node << "'s path");
			}
		}
	}
}

void GraphPathCalculator::getNavigationPath(
		std::vector<geometry_msgs::PoseStamped> &path,
		rrt_nbv_exploration_msgs::Graph &rrg, int goal_node,
		geometry_msgs::Point robot_pose) {
//	ROS_INFO_STREAM("get nav path from " << rrg.nearest_node << " to " << goal_node);
	ros::Time timestamp = ros::Time::now();
	//add robot position with orientation towards nearest node as first path node
//	geometry_msgs::PoseStamped path_pose;
//	path_pose.header.frame_id = "map";
//	path_pose.header.stamp = timestamp;
//	path_pose.pose.position = robot_pose;
//	path_pose.pose.position.z =
//			rrg.nodes[rrg.nodes[goal_node].pathToRobot[0]].position.z;
//	double yaw = atan2(
//			rrg.nodes[rrg.nodes[goal_node].pathToRobot[0]].position.y
//					- robot_pose.y,
//			rrg.nodes[rrg.nodes[goal_node].pathToRobot[0]].position.x
//					- robot_pose.x);
//	tf2::Quaternion quaternion_robot;
//	quaternion_robot.setRPY(0, 0, yaw);
//	quaternion_robot.normalize();
//	path_pose.pose.orientation = tf2::toMsg(quaternion_robot);
//	path.push_back(path_pose);
	if (rrg.nearest_node == goal_node) { //nearest node to robot and goal node are the same
		//add poses between robot and node
		geometry_msgs::PoseStamped path_pose;
		path_pose.header.frame_id = "map";
		path_pose.header.stamp = timestamp;
		robot_pose.z = rrg.nodes[goal_node].position.z;
		double yaw = atan2(rrg.nodes[goal_node].position.y - robot_pose.y,
				rrg.nodes[goal_node].position.x - robot_pose.x);
		tf2::Quaternion quaternion;
		quaternion.setRPY(0, 0, yaw);
		quaternion.normalize();
		addInterNodes(path, robot_pose, rrg.nodes[goal_node].position,
				tf2::toMsg(quaternion), yaw);
		//use best yaw for orientation at goal node
		path_pose.pose.position = rrg.nodes[goal_node].position;
		tf2::Quaternion quaternion_goal;
		quaternion_goal.setRPY(0, 0,
		M_PI * rrg.nodes[goal_node].best_yaw / 180.0);
		quaternion_goal.normalize();
		path_pose.pose.orientation = tf2::toMsg(quaternion_goal);
		path.push_back(path_pose);
	} else { //build a path from start to root and from goal to root until they meet

		//compare robot distance to second node on path with edge length between first and second node to decide if first is discarded
		std::vector<int> pathToRobot = rrg.nodes[goal_node].pathToRobot;
		if (pathToRobot.size() >= 2) {
			double distance_first_second_squared = pow(
					rrg.nodes[pathToRobot.at(0)].position.x
							- rrg.nodes[pathToRobot.at(1)].position.x, 2)
					+ pow(
							rrg.nodes[pathToRobot.at(0)].position.y
									- rrg.nodes[pathToRobot.at(1)].position.y,
							2);
			double distance_second_squared = pow(
					robot_pose.x - rrg.nodes[pathToRobot.at(1)].position.x, 2)
					+ pow(
							robot_pose.y
									- rrg.nodes[pathToRobot.at(1)].position.y,
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
			path_pose.pose.position = rrg.nodes[i].position;
			tf2::Quaternion quaternion;
			double yaw;
			if (&i == &pathToRobot.front()) { //if node is first element in list, add poses between robot and node
				geometry_msgs::Pose robot_pose = getRobotPose();
				robot_pose.position.z = rrg.nodes[i].position.z;
				yaw = atan2(rrg.nodes[i].position.y - robot_pose.position.y,
						rrg.nodes[i].position.x - robot_pose.position.x);
				quaternion.setRPY(0, 0, yaw);
				quaternion.normalize();
				addInterNodes(path, robot_pose.position, rrg.nodes[i].position,
						tf2::toMsg(quaternion), yaw);
			}
			if (&i != &pathToRobot.back()) //if node is not last element in list, get orientation between this node and the next
				yaw = atan2(
						rrg.nodes[*(&i + 1)].position.y
								- rrg.nodes[i].position.y,
						rrg.nodes[*(&i + 1)].position.x
								- rrg.nodes[i].position.x);
			else
				//last node in list, use best yaw for orientation
				yaw = M_PI * rrg.nodes[i].best_yaw / 180.0;
			quaternion.setRPY(0, 0, yaw);
			quaternion.normalize();
			path_pose.pose.orientation = tf2::toMsg(quaternion);
			path.push_back(path_pose);
			//add in between nodes
			if (&i != &pathToRobot.back()) { //add in between node
				addInterNodes(path, rrg.nodes[i].position,
						rrg.nodes[*(&i + 1)].position, tf2::toMsg(quaternion),
						yaw);
			}
		}
	}
}

void GraphPathCalculator::addInterNodes(
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

bool GraphPathCalculator::neighbourNodes(rrt_nbv_exploration_msgs::Graph &rrg,
		int startNode, int endNode) {
	for (int edge : rrg.nodes[startNode].edges) {
		if ((rrg.edges[edge].first_node == startNode
				&& rrg.edges[edge].second_node == endNode)
				|| (rrg.edges[edge].first_node == endNode
						&& rrg.edges[edge].second_node == startNode))
			return true;
	}
	return false;
}

}
