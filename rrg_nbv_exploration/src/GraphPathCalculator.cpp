#include <rrg_nbv_exploration/GraphPathCalculator.h>

namespace rrg_nbv_exploration {
GraphPathCalculator::GraphPathCalculator() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");
	_last_robot_yaw = 0;
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

int GraphPathCalculator::getRobotYaw(const geometry_msgs::Pose &robot_pos) {
	//get current robot orientation (yaw) for heading change calculation
	tf2::Quaternion q(robot_pos.orientation.x, robot_pos.orientation.y,
			robot_pos.orientation.z, robot_pos.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	int robot_yaw = (int) ((yaw * 180 / M_PI)); //all headings in degrees [0,360)
	if (robot_yaw < 0)
		robot_yaw += 360;

	return robot_yaw;
}

void GraphPathCalculator::updatePathsToRobot(int startNode,
		rrg_nbv_exploration_msgs::Graph &rrg, geometry_msgs::Pose robot_pos,
		bool reset) {
	int robot_yaw = getRobotYaw(robot_pos); //get current robot orientation (yaw) for heading change calculation
	_last_robot_yaw = robot_yaw;
	//run Dijkstra on RRG and assign distance and path to each node
	std::set<std::pair<double, int>> node_queue;
	if (reset) { //robot moved and is currently at startNode's position
		for (auto &node : rrg.nodes) {
			node.distance_to_robot = std::numeric_limits<double>::infinity();
			node.path_to_robot.clear();
			node.heading_change_to_robot = 0;
			node.traversability_cost_to_robot = 0.0;
			node.heading_in = 0;
		}
		rrg.nodes[startNode].distance_to_robot = 0;
		rrg.nodes[startNode].path_to_robot.push_back(startNode);
		rrg.nodes[startNode].heading_change_to_robot = 0;
		rrg.nodes[startNode].traversability_cost_to_robot =
				rrg.nodes[startNode].traversability_cost;
		rrg.nodes[startNode].heading_in = robot_yaw;
		if (rrg.nodes[startNode].gain > 0)
			rrg.nodes[startNode].heading_change_to_robot_best_view =
					(double) getAbsoluteAngleDiff(
							rrg.nodes[startNode].heading_in,
							rrg.nodes[startNode].best_yaw)
							/ (180.0
									* (double) rrg.nodes[startNode].path_to_robot.size());
		else
			rrg.nodes[startNode].heading_change_to_robot_best_view = 0.0;
		rrg.longest_distance_to_robot = 0;
	}
	node_queue.insert(
			std::make_pair(rrg.nodes[startNode].distance_to_robot, startNode));
	while (!node_queue.empty()) {
		double current_distance = node_queue.begin()->first;
		int current_node = node_queue.begin()->second;
		node_queue.erase(node_queue.begin());
		for (auto edge : rrg.nodes[current_node].edges) {
			int neighbor_node =
					rrg.edges[edge].first_node == current_node ?
							rrg.edges[edge].second_node :
							rrg.edges[edge].first_node;
			double neighbor_distance = rrg.edges[edge].length;
			double total_distance = current_distance + neighbor_distance;
			if (total_distance < rrg.nodes[neighbor_node].distance_to_robot) {
				node_queue.erase(
						std::make_pair(
								rrg.nodes[neighbor_node].distance_to_robot,
								neighbor_node));
				rrg.nodes[neighbor_node].distance_to_robot = total_distance;
				if (total_distance > rrg.longest_distance_to_robot) {
					rrg.longest_distance_to_robot = total_distance;
				}
				rrg.nodes[neighbor_node].path_to_robot =
						rrg.nodes[current_node].path_to_robot;
				rrg.nodes[neighbor_node].path_to_robot.push_back(neighbor_node);
				int current_heading_out = 0;
				if (rrg.edges[edge].second_node == current_node) { //edge yaw is from first to second node
					current_heading_out =
							rrg.edges[edge].yaw >= 180 ?
									rrg.edges[edge].yaw - 180 :
									rrg.edges[edge].yaw + 180;
				} else {
					current_heading_out = rrg.edges[edge].yaw;
				}
				rrg.nodes[neighbor_node].heading_in = current_heading_out;
				rrg.nodes[neighbor_node].heading_change_to_robot =
						rrg.nodes[current_node].heading_change_to_robot
								+ getAbsoluteAngleDiff(
										rrg.nodes[current_node].heading_in,
										current_heading_out); //calculate heading cost from current to neighbor node
				if (rrg.nodes[neighbor_node].gain > 0)
					rrg.nodes[neighbor_node].heading_change_to_robot_best_view =
							(double) (rrg.nodes[neighbor_node].heading_change_to_robot
									+ getAbsoluteAngleDiff(
											rrg.nodes[neighbor_node].heading_in,
											rrg.nodes[neighbor_node].best_yaw))
									/ (180.0
											* (double) rrg.nodes[neighbor_node].path_to_robot.size());
				else
					rrg.nodes[neighbor_node].heading_change_to_robot_best_view =
							0.0;
				rrg.nodes[neighbor_node].traversability_cost_to_robot =
						rrg.nodes[current_node].traversability_cost_to_robot
								+ rrg.nodes[neighbor_node].traversability_cost
								+ rrg.edges[edge].traversability_cost;
				node_queue.insert(
						std::make_pair(
								rrg.nodes[neighbor_node].distance_to_robot,
								neighbor_node));
			}
		}
	}
}

bool GraphPathCalculator::updateHeadingToRobot(int startNode,
		rrg_nbv_exploration_msgs::Graph &rrg, geometry_msgs::Pose robot_pos) {
	int robot_yaw = getRobotYaw(robot_pos);
	if (robot_yaw == _last_robot_yaw) //only calculate new heading if yaw changed
		return false;
	rrg.nodes[startNode].heading_in = robot_yaw;
	if (rrg.nodes[startNode].gain > 0)
		rrg.nodes[startNode].heading_change_to_robot_best_view =
				(double) getAbsoluteAngleDiff(rrg.nodes[startNode].heading_in,
						rrg.nodes[startNode].best_yaw)
						/ (180.0
								* (double) rrg.nodes[startNode].path_to_robot.size());
	else
		rrg.nodes[startNode].heading_change_to_robot_best_view = 0.0;
	_last_robot_yaw = robot_yaw;
	std::map<int, int> first_nodes; //nodes with a direct edge to the start node (index, heading difference)
	for (int edge : rrg.nodes[startNode].edges) { //find nodes with direct edge to start node and calculate the difference
		int first_node = -1;
		int edge_yaw = -1;
		if (rrg.edges[edge].first_node == startNode) {
			first_node = rrg.edges[edge].second_node;
			edge_yaw = rrg.edges[edge].yaw;
		} else { //reverse yaw when travelling edge the opposite way
			first_node = rrg.edges[edge].first_node;
			edge_yaw =
					rrg.edges[edge].yaw >= 180 ?
							rrg.edges[edge].yaw - 180 :
							rrg.edges[edge].yaw + 180;
		}
		int new_yaw = getAbsoluteAngleDiff(edge_yaw,
				rrg.nodes[startNode].heading_in);
		//get signed difference between angles
		int difference = new_yaw
				- rrg.nodes[first_node].heading_change_to_robot;
		difference = (difference + 180) % 360 - 180;
		first_nodes.insert(std::make_pair(first_node, difference));
	}
	for (int i = 0; i < rrg.node_counter; i++) { //update all node headings to robot depending on the first node in their particular path
		if (rrg.nodes[i].path_to_robot.size() >= 2) {
			rrg.nodes[i].heading_change_to_robot =
					rrg.nodes[i].heading_change_to_robot
							+ first_nodes.at(rrg.nodes[i].path_to_robot[1]);
			if (rrg.nodes[i].gain > 0)
				rrg.nodes[i].heading_change_to_robot_best_view =
						rrg.nodes[i].heading_change_to_robot_best_view
								+ ((double) first_nodes.at(
										rrg.nodes[i].path_to_robot[1])
										/ (180.0
												* (double) rrg.nodes[i].path_to_robot.size()));
		}
	}
	return true;
}

int GraphPathCalculator::getAbsoluteAngleDiff(int x, int y) {
	return 180 - abs(abs(x - y) - 180);
}

void GraphPathCalculator::getNavigationPath(
		std::vector<geometry_msgs::PoseStamped> &path,
		rrg_nbv_exploration_msgs::Graph &rrg, int goal_node,
		geometry_msgs::Point robot_pose) {
//	ROS_INFO_STREAM("get nav path from " << rrg.nearest_node << " to " << goal_node);
	ros::Time timestamp = ros::Time::now();
//add robot position with orientation towards nearest node as first path node
//	geometry_msgs::PoseStamped path_pose;
//	path_pose.header.frame_id = "map";
//	path_pose.header.stamp = timestamp;
//	path_pose.pose.position = robot_pose;
//	path_pose.pose.position.z =
//			rrg.nodes[rrg.nodes[goal_node].path_to_robot[0]].position.z;
//	double yaw = atan2(
//			rrg.nodes[rrg.nodes[goal_node].path_to_robot[0]].position.y
//					- robot_pose.y,
//			rrg.nodes[rrg.nodes[goal_node].path_to_robot[0]].position.x
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
		std::vector<int> path_to_robot = rrg.nodes[goal_node].path_to_robot;
		if (path_to_robot.size() >= 2) {
			double distance_first_second_squared = pow(
					rrg.nodes[path_to_robot.at(0)].position.x
							- rrg.nodes[path_to_robot.at(1)].position.x, 2)
					+ pow(
							rrg.nodes[path_to_robot.at(0)].position.y
									- rrg.nodes[path_to_robot.at(1)].position.y,
							2);
			double distance_second_squared = pow(
					robot_pose.x - rrg.nodes[path_to_robot.at(1)].position.x, 2)
					+ pow(
							robot_pose.y
									- rrg.nodes[path_to_robot.at(1)].position.y,
							2);
			if (distance_first_second_squared > distance_second_squared) {
				//remove first node from path since it leads backwards on the path
				path_to_robot.erase(path_to_robot.begin());
			}
		}
		for (auto &i : path_to_robot) { //iterate through nodes in path and add as waypoints for path
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.frame_id = "map";
			path_pose.header.stamp = timestamp;
			path_pose.pose.position = rrg.nodes[i].position;
			tf2::Quaternion quaternion;
			double yaw;
			if (&i == &path_to_robot.front()) { //if node is first element in list, add poses between robot and node
				geometry_msgs::Pose robot_pose = getRobotPose();
				robot_pose.position.z = rrg.nodes[i].position.z;
				yaw = atan2(rrg.nodes[i].position.y - robot_pose.position.y,
						rrg.nodes[i].position.x - robot_pose.position.x);
				quaternion.setRPY(0, 0, yaw);
				quaternion.normalize();
				addInterNodes(path, robot_pose.position, rrg.nodes[i].position,
						tf2::toMsg(quaternion), yaw);
			}
			if (&i != &path_to_robot.back()) //if node is not last element in list, get orientation between this node and the next
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
			if (&i != &path_to_robot.back()) { //add in between node
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

bool GraphPathCalculator::neighbourNodes(rrg_nbv_exploration_msgs::Graph &rrg,
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
