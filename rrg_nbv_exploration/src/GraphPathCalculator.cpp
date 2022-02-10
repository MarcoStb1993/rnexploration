#include <rrg_nbv_exploration/GraphPathCalculator.h>

namespace rrg_nbv_exploration {
GraphPathCalculator::GraphPathCalculator() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");
	private_nh.param("sensor_horizontal_fov", _sensor_horizontal_fov, 360);
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
			node.traversability_cost_to_robot = 0.0;
			node.traversability_weight_to_robot = 0.0;
			node.heading_in = 0;
			node.heading_change_to_robot = 0;
			node.heading_change_to_robot_best_view = 0;
			node.radii_to_robot = 0;
		}

		rrg.nodes[startNode].distance_to_robot = 0;
		rrg.nodes[startNode].path_to_robot.push_back(startNode);
		rrg.nodes[startNode].heading_change_to_robot = 0;
		rrg.nodes[startNode].traversability_cost_to_robot =
				rrg.nodes[startNode].traversability_cost;
		rrg.nodes[startNode].traversability_weight_to_robot =
				rrg.nodes[startNode].traversability_weight;
		rrg.nodes[startNode].heading_in = robot_yaw;
		rrg.nodes[startNode].radii_to_robot = rrg.nodes[startNode].radius;
		setHeadingChangeToBestView(startNode, rrg);
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
				rrg.nodes[neighbor_node].radii_to_robot =
						rrg.nodes[current_node].radii_to_robot
								+ rrg.nodes[neighbor_node].radius;
				rrg.nodes[neighbor_node].heading_in = current_heading_out;
				rrg.nodes[neighbor_node].heading_change_to_robot =
						rrg.nodes[current_node].heading_change_to_robot
								+ getAbsoluteAngleDiff(
										rrg.nodes[current_node].heading_in,
										current_heading_out); //calculate heading cost from current to neighbor node
				setHeadingChangeToBestView(neighbor_node, rrg);
				rrg.nodes[neighbor_node].traversability_cost_to_robot =
						rrg.nodes[current_node].traversability_cost_to_robot
								+ rrg.nodes[neighbor_node].traversability_cost
								+ rrg.edges[edge].traversability_cost;
				rrg.nodes[neighbor_node].traversability_weight_to_robot =
						rrg.nodes[current_node].traversability_weight_to_robot
								+ rrg.nodes[neighbor_node].traversability_weight
								+ rrg.edges[edge].traversability_weight;
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
	setHeadingChangeToBestView(startNode, rrg);
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

			setHeadingChangeToBestView(i, rrg);
		}
	}
	return true;
}

int GraphPathCalculator::getAbsoluteAngleDiff(int x, int y) {
	return 180 - abs(abs(x - y) - 180);
}

void GraphPathCalculator::setHeadingChangeToBestView(int node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	if (rrg.nodes[node].gain > 0) {
		rrg.nodes[node].heading_change_to_robot_best_view =
				(double) (getAbsoluteAngleDiff(rrg.nodes[node].heading_in,
						rrg.nodes[node].best_yaw));
	} else
		rrg.nodes[node].heading_change_to_robot_best_view = 0;
}

void GraphPathCalculator::getNavigationPath(
		std::vector<geometry_msgs::PoseStamped> &path,
		rrg_nbv_exploration_msgs::Graph &rrg, int goal_node,
		geometry_msgs::Point robot_pose) {
	if (rrg.nearest_node == goal_node) { //nearest node to robot and goal node are the same
		getPathFromRobotToNode(robot_pose, goal_node, rrg, path);
	} else { //build a path from start to root and from goal to root until they meet
		//compare robot distance to second node on path with edge length between first and second node to decide if first is discarded
		ros::Time timestamp = ros::Time::now();
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
				path_to_robot.erase(path_to_robot.begin()); //remove first node from path since it leads backwards on the path
			}
		}
		if (path_to_robot.size() < 2) { //only path from robot to goal node necessary
			getPathFromRobotToNode(robot_pose, goal_node, rrg, path);
			return;
		}
		for (auto &i : path_to_robot) { //iterate through nodes in path and add as waypoints for path
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.frame_id = "map";
			path_pose.header.stamp = timestamp;
			path_pose.pose.position = rrg.nodes[i].position;
			tf2::Quaternion quaternion;
			double yaw, distance;
			if (i == path_to_robot.front()) { //if node is first element in list, add poses between robot and node
				robot_pose.z = rrg.nodes[i].position.z;
				yaw = atan2(rrg.nodes[i].position.y - robot_pose.y,
						rrg.nodes[i].position.x - robot_pose.x);
				distance = sqrt(
						pow(rrg.nodes[i].position.x - robot_pose.x, 2)
								+ pow(rrg.nodes[i].position.y - robot_pose.y,
										2));
				quaternion.setRPY(0, 0, yaw);
				quaternion.normalize();
				addInterNodes(path, robot_pose, rrg.nodes[i].position,
						tf2::toMsg(quaternion), yaw, distance);
			}
			if (i != path_to_robot.back()) { //if node is not last element in list, get orientation between this node and the next
				int heading;
				getHeadingBetweenNodes(i, *(&i + 1), heading, distance, rrg);
				yaw = M_PI * heading / 180;
			} else {
				if (_sensor_horizontal_fov == 360) { //use yaw from incoming edge for 360 degrees FoV
					int heading;
					getHeadingBetweenNodes(*(&i - 1), i, heading, distance,
							rrg);
					yaw = M_PI * heading / 180;
				} else
					//last node in list, use best yaw for orientation
					yaw = M_PI * rrg.nodes[i].best_yaw / 180.0;
			}
			quaternion.setRPY(0, 0, yaw);
			quaternion.normalize();
			path_pose.pose.orientation = tf2::toMsg(quaternion);
			path.push_back(path_pose);
			//add in between nodes
			if (i != path_to_robot.back()) { //add in between node
				addInterNodes(path, rrg.nodes[i].position,
						rrg.nodes[*(&i + 1)].position, tf2::toMsg(quaternion),
						yaw, distance);
			}
		}
	}
}

void GraphPathCalculator::getPathFromRobotToNode(
		geometry_msgs::Point robot_pose, int goal_node,
		rrg_nbv_exploration_msgs::Graph &rrg,
		std::vector<geometry_msgs::PoseStamped> &path) {
	ros::Time timestamp = ros::Time::now();
	geometry_msgs::PoseStamped path_pose;
	path_pose.header.frame_id = "map";
	path_pose.header.stamp = timestamp;
	robot_pose.z = rrg.nodes[goal_node].position.z;
	double yaw = atan2(rrg.nodes[goal_node].position.y - robot_pose.y,
			rrg.nodes[goal_node].position.x - robot_pose.x);
	double distance = sqrt(
			pow(rrg.nodes[goal_node].position.x - robot_pose.x, 2)
					+ pow(rrg.nodes[goal_node].position.y - robot_pose.y, 2));
	tf2::Quaternion quaternion;
	quaternion.setRPY(0, 0, yaw);
	quaternion.normalize();
	addInterNodes(path, robot_pose, rrg.nodes[goal_node].position,
			tf2::toMsg(quaternion), yaw, distance);	//add poses between robot and node
	path_pose.pose.position = rrg.nodes[goal_node].position;
	if (_sensor_horizontal_fov == 360) { //use best yaw for orientation at goal node or keep yaw for 360 horizontal FoV
		path_pose.pose.orientation = tf2::toMsg(quaternion);
	} else {
		tf2::Quaternion quaternion_goal;
		quaternion_goal.setRPY(0, 0,
		M_PI * rrg.nodes[goal_node].best_yaw / 180.0);
		quaternion_goal.normalize();
		path_pose.pose.orientation = tf2::toMsg(quaternion_goal);
	}
	path.push_back(path_pose);
}

bool GraphPathCalculator::getHeadingBetweenNodes(int start_node, int end_node,
		int &yaw, double &distance, rrg_nbv_exploration_msgs::Graph &rrg) {
	for (int edge : rrg.nodes[start_node].edges) {
		if (rrg.edges[edge].first_node == end_node
				&& rrg.edges[edge].second_node == start_node) { //edge yaw is from first to second node
			yaw = rrg.edges[edge].yaw >= 180 ?
					rrg.edges[edge].yaw - 180 : rrg.edges[edge].yaw + 180;
			distance = rrg.edges[edge].length;
			return true;
		} else if (rrg.edges[edge].first_node == start_node
				&& rrg.edges[edge].second_node == end_node) {
			yaw = rrg.edges[edge].yaw;
			distance = rrg.edges[edge].length;
			return true;
		}
	}
	ROS_WARN_STREAM(
			"Get Heading failed for start node " << start_node << " and end node " << end_node);
	return false;
}

void GraphPathCalculator::addInterNodes(
		std::vector<geometry_msgs::PoseStamped> &path,
		geometry_msgs::Point start, geometry_msgs::Point end,
		geometry_msgs::Quaternion orientation, double yaw, double distance) {
	geometry_msgs::PoseStamped path_pose;
	path_pose.header.frame_id = "map";
	path_pose.header.stamp = ros::Time::now();
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

int GraphPathCalculator::determineGoalYaw(int current_goal,
		rrg_nbv_exploration_msgs::Graph &rrg, geometry_msgs::Point robot_pose) {
	if (_sensor_horizontal_fov == 360) { //use yaw from incoming edge for 360 degrees FoV
		int yaw = 0;
		double distance;
		if (rrg.nodes[current_goal].path_to_robot.size() > 1
				&& !getHeadingBetweenNodes(
						rrg.nodes[current_goal].path_to_robot[rrg.nodes[current_goal].path_to_robot.size()
								- 2], current_goal, yaw, distance, rrg)) { //check if current goal is goal nearest to robot
			double yaw = atan2(
					rrg.nodes[current_goal].position.y - robot_pose.y,
					rrg.nodes[current_goal].position.x - robot_pose.x);
			yaw *= 180 / M_PI;
		}
		return yaw;
	}
	//use best yaw for orientation
	return rrg.nodes[current_goal].best_yaw;
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
