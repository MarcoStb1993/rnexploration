#include <rrg_nbv_exploration/GraphPathCalculator.h>

namespace rrg_nbv_exploration {
GraphPathCalculator::GraphPathCalculator() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,
			"base_footprint");
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("sensor_horizontal_fov", _sensor_horizontal_fov, 360);
	private_nh.param("radius_factor", _radius_factor, 1.0);
	private_nh.param("distance_factor", _distance_factor, 1.0);
	private_nh.param("heading_factor", _heading_factor, 1.0);
	private_nh.param("traversability_factor", _traversability_factor, 1.0);
	private_nh.param("inflation_active", _inflation_active, true);
	private_nh.param("grid_map_occupied", _grid_map_occupied, 100);
	_last_robot_yaw = 0;
	_robot_radius_squared = pow(_robot_radius, 2);
	_last_nearest_node = 0;
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

void GraphPathCalculator::updatePathsToRobot(int start_node,
		rrg_nbv_exploration_msgs::Graph &rrg, geometry_msgs::Pose robot_pos,
		bool reset, std::list<int> &nodes_to_update,
		bool &added_node_to_update) {
	int robot_yaw = getRobotYaw(robot_pos); //get current robot orientation (yaw) for heading change calculation
	_last_robot_yaw = robot_yaw;
	//run Dijkstra on RRG and assign distance and path to each node
	std::set<int> node_queue;
	if (reset) { //robot moved and is currently at startNode's position
		for (auto &node : rrg.nodes) {
			node.distance_to_robot = std::numeric_limits<double>::infinity();
			node.path_to_robot.clear();
			node.traversability_cost_to_robot = 0.0;
			node.traversability_weight_to_robot = 1.0;
			node.heading_change_to_robot = 0;
			node.heading_change_to_robot_best_view =
					std::numeric_limits<int>::max();
			node.radii_to_robot = 0;
			node.cost_function = std::numeric_limits<double>::infinity();
		}

		initializeStartingNode(start_node, robot_pos, robot_yaw, rrg);
		if (_last_nearest_node >= 0
				&& findExistingEdge(rrg, start_node, _last_nearest_node) >= 0
				&& rrg.nodes[_last_nearest_node].status
						!= rrg_nbv_exploration_msgs::Node::FAILED) {
			initializeStartingNode(_last_nearest_node, robot_pos, robot_yaw,
					rrg);
			node_queue.insert(_last_nearest_node);
		}
	}
	node_queue.insert(start_node);
	findBestRoutes(node_queue, rrg, reset, nodes_to_update,
			added_node_to_update);
	if (reset)
		_last_nearest_node = start_node;
}

bool GraphPathCalculator::updateHeadingToRobot(int start_node,
		rrg_nbv_exploration_msgs::Graph &rrg, geometry_msgs::Pose &robot_pos,
		int next_node, int edge_to_next_node,
		double distance_to_nearest_node_squared,
		std::list<int> &nodes_to_update, bool &added_node_to_update) {
	int robot_yaw = getRobotYaw(robot_pos);
	if (robot_yaw == _last_robot_yaw) //only calculate new heading if yaw changed
		return false;
	_last_robot_yaw = robot_yaw;
	for (auto &node : rrg.nodes) { //Reset cost functions
		node.distance_to_robot = std::numeric_limits<double>::infinity();
		node.path_to_robot.clear();
		node.traversability_cost_to_robot = 0.0;
		node.traversability_weight_to_robot = 1.0;
		node.heading_in = 0;
		node.heading_change_to_robot = 0;
		node.heading_change_to_robot_best_view =
				std::numeric_limits<int>::max();
		node.radii_to_robot = 0;
		node.cost_function = std::numeric_limits<double>::infinity();
	}
	initializeStartingNode(start_node, robot_pos, robot_yaw, rrg);
	rrg.nodes[start_node].cost_function = calculateCostFunction(
			rrg.nodes[start_node]);
	std::set<int> node_queue; //queue to nodes that were updates
	if (distance_to_nearest_node_squared <= _robot_radius_squared) { //robot in proximity to nearest node
		for (int edge : rrg.nodes[start_node].edges) { //find nodes with direct edge to start node and calculate the difference
			int direct_neighbor_node = -1;
			int edge_yaw = -1;
			if (rrg.edges[edge].first_node == start_node) {
				direct_neighbor_node = rrg.edges[edge].second_node;
				edge_yaw = rrg.edges[edge].yaw;
			} else { //reverse yaw when travelling edge the opposite way
				direct_neighbor_node = rrg.edges[edge].first_node;
				edge_yaw =
						rrg.edges[edge].yaw >= 180 ?
								rrg.edges[edge].yaw - 180 :
								rrg.edges[edge].yaw + 180;
			}
			if (rrg.nodes[direct_neighbor_node].status
					!= rrg_nbv_exploration_msgs::Node::FAILED) {
				int direct_neighbor_heading_change_to_robot =
						getAbsoluteAngleDiff(edge_yaw, robot_yaw);
				rrg.nodes[direct_neighbor_node].heading_in = edge_yaw;
				rrg.nodes[direct_neighbor_node].heading_change_to_robot =
						direct_neighbor_heading_change_to_robot;
				rrg.nodes[direct_neighbor_node].heading_change_to_robot_best_view =
						calculateHeadingChangeToBestView(
								rrg.nodes[direct_neighbor_node]);
				rrg.nodes[direct_neighbor_node].distance_to_robot =
						rrg.nodes[start_node].distance_to_robot
								+ rrg.edges[edge].length;
				rrg.nodes[direct_neighbor_node].path_to_robot =
						rrg.nodes[start_node].path_to_robot;
				rrg.nodes[direct_neighbor_node].path_to_robot.push_back(
						direct_neighbor_node);
				rrg.nodes[direct_neighbor_node].traversability_cost_to_robot =
						rrg.nodes[start_node].traversability_cost_to_robot
								+ rrg.nodes[direct_neighbor_node].traversability_cost
								+ rrg.edges[edge].traversability_cost;
				rrg.nodes[direct_neighbor_node].traversability_weight_to_robot =
						rrg.nodes[start_node].traversability_weight_to_robot
								+ rrg.nodes[direct_neighbor_node].traversability_weight
								+ rrg.edges[edge].traversability_weight;
				rrg.nodes[direct_neighbor_node].radii_to_robot =
						rrg.nodes[start_node].radii_to_robot
								+ rrg.nodes[direct_neighbor_node].radius;
				rrg.nodes[direct_neighbor_node].cost_function =
						calculateCostFunction(rrg.nodes[direct_neighbor_node]);
				node_queue.insert(direct_neighbor_node);
			}
		}
	} else {
		node_queue.insert(start_node);
		if (next_node != -1 && edge_to_next_node != -1
				&& rrg.nodes[next_node].status
						!= rrg_nbv_exploration_msgs::Node::FAILED) { // ignore start node for distance, path, traversability and radii
			initializeStartingNode(next_node, robot_pos, robot_yaw, rrg);
			rrg.nodes[next_node].distance_to_robot =
					rrg.edges[edge_to_next_node].length;
			rrg.nodes[next_node].traversability_cost_to_robot +=
					rrg.edges[edge_to_next_node].traversability_cost;
			rrg.nodes[next_node].traversability_weight_to_robot +=
					rrg.edges[edge_to_next_node].traversability_weight;
			node_queue.insert(next_node);
		}
	}
	findBestRoutes(node_queue, rrg, true, nodes_to_update,
			added_node_to_update);
	return true;
}

void GraphPathCalculator::initializeStartingNode(int node,
		const geometry_msgs::Pose &robot_pos, int robot_yaw,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	int yaw_to_start_node = (int) ((atan2(
			rrg.nodes[node].position.y - robot_pos.position.y,
			rrg.nodes[node].position.x - robot_pos.position.x) * 180.0 / M_PI));
	if (yaw_to_start_node < 0)
		yaw_to_start_node += 360;

	rrg.nodes[node].heading_in = yaw_to_start_node;
	rrg.nodes[node].heading_change_to_robot = getAbsoluteAngleDiff(
			yaw_to_start_node, robot_yaw);
	rrg.nodes[node].heading_change_to_robot_best_view =
			calculateHeadingChangeToBestView(rrg.nodes[node]);
	rrg.nodes[node].distance_to_robot = 0;
	rrg.nodes[node].path_to_robot.push_back(node);
	rrg.nodes[node].traversability_cost_to_robot =
			rrg.nodes[node].traversability_cost;
	rrg.nodes[node].traversability_weight_to_robot =
			rrg.nodes[node].traversability_weight;
	rrg.nodes[node].radii_to_robot = rrg.nodes[node].radius;
	rrg.nodes[node].cost_function = calculateCostFunction(rrg.nodes[node]);
}

void GraphPathCalculator::findBestRoutes(std::set<int> node_queue,
		rrg_nbv_exploration_msgs::Graph &rrg, bool reset,
		std::list<int> &nodes_to_update, bool &added_node_to_update) {
	while (!node_queue.empty()) {
		int current_node = *node_queue.begin();
		node_queue.erase(node_queue.begin());
		for (auto edge : rrg.nodes[current_node].edges) {
			int neighbor_node_index =
					rrg.edges[edge].first_node == current_node ?
							rrg.edges[edge].second_node :
							rrg.edges[edge].first_node;
			if (isNodeInPath(neighbor_node_index, current_node, rrg)
					|| rrg.nodes[neighbor_node_index].status
							== rrg_nbv_exploration_msgs::Node::FAILED) { //don't allow loops and failed nodes in path
				continue;
			}
			rrg_nbv_exploration_msgs::Node node_with_new_connection =
					calculateCostFunctionForConnection(current_node,
							neighbor_node_index, edge, rrg);
			if (node_with_new_connection.cost_function
					< rrg.nodes[neighbor_node_index].cost_function) {
				if (!reset
						&& std::isinf(
								rrg.nodes[neighbor_node_index].cost_function)) { //add previously unreachable node to list of nodes to update
					nodes_to_update.push_back(neighbor_node_index);
					added_node_to_update = true;
				}
				rrg.nodes[neighbor_node_index].distance_to_robot =
						node_with_new_connection.distance_to_robot;
				rrg.nodes[neighbor_node_index].path_to_robot =
						node_with_new_connection.path_to_robot;
				rrg.nodes[neighbor_node_index].radii_to_robot =
						node_with_new_connection.radii_to_robot;
				rrg.nodes[neighbor_node_index].heading_in =
						node_with_new_connection.heading_in;
				rrg.nodes[neighbor_node_index].heading_change_to_robot =
						node_with_new_connection.heading_change_to_robot;
				rrg.nodes[neighbor_node_index].heading_change_to_robot_best_view =
						node_with_new_connection.heading_change_to_robot_best_view;
				rrg.nodes[neighbor_node_index].traversability_cost_to_robot =
						node_with_new_connection.traversability_cost_to_robot;
				rrg.nodes[neighbor_node_index].traversability_weight_to_robot =
						node_with_new_connection.traversability_weight_to_robot;
				rrg.nodes[neighbor_node_index].cost_function =
						node_with_new_connection.cost_function;
				node_queue.insert(neighbor_node_index);
			}
		}
	}
}

bool GraphPathCalculator::isNodeInPath(int neighbor_node_index,
		int current_node, rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto i : rrg.nodes[current_node].path_to_robot) {
		if (i == neighbor_node_index)
			return true;
	}
	return false;
}

double GraphPathCalculator::calculateCostFunction(
		rrg_nbv_exploration_msgs::Node &node) {
	double traversability = (double) node.traversability_cost_to_robot
			/ (double) node.traversability_weight_to_robot; //TODO: re-evaluate traversability cost
	double distance = node.distance_to_robot;
	double heading = (double) node.heading_change_to_robot_best_view / 180.0; //every half-turn costs as much as 1m of distance
	double cost_function = _distance_factor * distance
			+ _traversability_factor * traversability
			+ _heading_factor * heading;
	if (_inflation_active && _radius_factor > 0) {
		double radius = (node.radii_to_robot
				/ (double) node.path_to_robot.size() / _robot_radius) - 1.0;
		if (cost_function > 0)
			return (cost_function / (1.0 + _radius_factor * radius));
		else
			return (1.0 / (1.0 + _radius_factor * radius));
	} else {
		return cost_function;
	}
}

rrg_nbv_exploration_msgs::Node GraphPathCalculator::calculateCostFunctionForConnection(
		int neighbor_node, int node, int edge,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	rrg_nbv_exploration_msgs::Node updated_node;
	updated_node.distance_to_robot = rrg.nodes[neighbor_node].distance_to_robot
			+ rrg.edges[edge].length;
	updated_node.path_to_robot = rrg.nodes[neighbor_node].path_to_robot;
	updated_node.path_to_robot.push_back(node);
	updated_node.radii_to_robot = rrg.nodes[neighbor_node].radii_to_robot
			+ rrg.nodes[node].radius;
	if (rrg.edges[edge].second_node == neighbor_node) { //edge yaw is from first to second node
		updated_node.heading_in =
				rrg.edges[edge].yaw >= 180 ?
						rrg.edges[edge].yaw - 180 : rrg.edges[edge].yaw + 180;
	} else {
		updated_node.heading_in = rrg.edges[edge].yaw;
	}
	updated_node.heading_change_to_robot =
			rrg.nodes[neighbor_node].heading_change_to_robot
					+ getAbsoluteAngleDiff(rrg.nodes[neighbor_node].heading_in,
							updated_node.heading_in); //calculate heading cost from current to neighbor node
	updated_node.heading_change_to_robot_best_view =
			calculateHeadingChangeToBestView(updated_node);
	updated_node.traversability_cost_to_robot =
			rrg.nodes[neighbor_node].traversability_cost_to_robot
					+ rrg.nodes[node].traversability_cost
					+ rrg.edges[edge].traversability_cost;
	updated_node.traversability_weight_to_robot =
			rrg.nodes[neighbor_node].traversability_weight_to_robot
					+ rrg.nodes[node].traversability_weight
					+ rrg.edges[edge].traversability_weight;
	updated_node.cost_function = calculateCostFunction(updated_node);
	return updated_node;
}

void GraphPathCalculator::findBestConnectionForNode(
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::Node &node, geometry_msgs::Pose &robot_pos,
		bool new_node, std::vector<rrg_nbv_exploration_msgs::Edge> edges) {
	std::vector<rrg_nbv_exploration_msgs::Edge> node_edges;
	if (!new_node) { //get edges from given existing node
		for (auto i : node.edges) {
			node_edges.push_back(rrg.edges[i]);
		}
	} else { //use given edges for new node
		node_edges = edges;
	}
	for (auto &edge : node_edges) {
		if (new_node) { //store edges in RRG and references in the particular nodes
			edge.index = rrg.edge_counter++;
			node.edges.push_back(edge.index);
			node.edge_counter++;
			rrg.edges.push_back(edge);
			rrg.nodes[edge.first_node].edges.push_back(edge.index);
			rrg.nodes[edge.first_node].edge_counter++;
		}
		if (rrg.nodes[edge.first_node].status
				!= rrg_nbv_exploration_msgs::Node::FAILED
				&& !std::isinf(rrg.nodes[edge.first_node].cost_function)) {
			//only evaluate edge if it is not to a failed or unreachable node
			rrg_nbv_exploration_msgs::Node updated_node; //calculate potential cost if connected via this edge
			updated_node.distance_to_robot =
					rrg.nodes[edge.first_node].distance_to_robot + edge.length;
			updated_node.path_to_robot =
					rrg.nodes[edge.first_node].path_to_robot;
			updated_node.path_to_robot.push_back(node.index);
			updated_node.radii_to_robot =
					rrg.nodes[edge.first_node].radii_to_robot + node.radius;
			if (edge.first_node == rrg.nearest_node) {
				//calculate heading to new node directly from robot if edge to nearest node
				int yaw_to_start_node =
						(int) (((atan2(node.position.y - robot_pos.position.y,
								node.position.x - robot_pos.position.x) * 180.0
								/ M_PI)));
				if (yaw_to_start_node < 0)
					yaw_to_start_node += 360;

				updated_node.heading_in = yaw_to_start_node;
			} else
				updated_node.heading_in = edge.yaw;

			updated_node.heading_change_to_robot =
					rrg.nodes[edge.first_node].heading_change_to_robot
							+ getAbsoluteAngleDiff(
									rrg.nodes[edge.first_node].heading_in,
									updated_node.heading_in); //calculate heading cost from current to neighbor node
			updated_node.heading_change_to_robot_best_view =
					calculateHeadingChangeToBestView(updated_node);
			updated_node.traversability_cost_to_robot =
					rrg.nodes[edge.first_node].traversability_cost_to_robot
							+ node.traversability_cost
							+ edge.traversability_cost;
			updated_node.traversability_weight_to_robot =
					rrg.nodes[edge.first_node].traversability_weight_to_robot
							+ node.traversability_weight
							+ edge.traversability_weight;
			updated_node.cost_function = calculateCostFunction(updated_node);
			if (updated_node.cost_function < node.cost_function) {
				node.distance_to_robot = updated_node.distance_to_robot;
				node.path_to_robot = updated_node.path_to_robot;
				node.heading_in = updated_node.heading_in;
				node.heading_change_to_robot =
						updated_node.heading_change_to_robot;
				node.heading_change_to_robot_best_view =
						updated_node.heading_change_to_robot_best_view;
				node.radii_to_robot = updated_node.radii_to_robot;
				node.traversability_cost_to_robot =
						updated_node.traversability_cost_to_robot;
				node.traversability_weight_to_robot =
						updated_node.traversability_weight_to_robot;
				node.cost_function = updated_node.cost_function;
			}
		}
	}
}

int GraphPathCalculator::getAbsoluteAngleDiff(int x, int y) {
	return 180 - abs(abs(x - y) - 180);
}

int GraphPathCalculator::calculateHeadingChangeToBestView(
		rrg_nbv_exploration_msgs::Node &node) {
	if (node.gain > 0 && _sensor_horizontal_fov < 360)
		return node.heading_change_to_robot
				+ getAbsoluteAngleDiff(node.heading_in, node.best_yaw);
	else
		return node.heading_change_to_robot;
}

int GraphPathCalculator::findExistingEdge(rrg_nbv_exploration_msgs::Graph &rrg,
		int node, int neighbor_node) {
	for (auto edge : rrg.nodes[node].edges) {
		if (rrg.edges[edge].first_node == neighbor_node
				|| rrg.edges[edge].second_node == neighbor_node)
			return edge; //connection already present
	}
	return -1;
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
		rrg_nbv_exploration_msgs::Graph &rrg, geometry_msgs::Point robot_pose,
		bool homing) {
	int yaw = rrg.nodes[current_goal].best_yaw; //normally use best yaw for orientation
	if (_sensor_horizontal_fov == 360 || homing) { //use yaw from incoming edge for 360 degrees FoV	or when going back to the start
		double distance;
		if (rrg.nodes[current_goal].path_to_robot.size() < 2
				|| !getHeadingBetweenNodes(
						rrg.nodes[current_goal].path_to_robot[rrg.nodes[current_goal].path_to_robot.size()
								- 2], current_goal, yaw, distance, rrg)) { //check if current goal is goal nearest to robot
			yaw = (int) (atan2(
					rrg.nodes[current_goal].position.y - robot_pose.y,
					rrg.nodes[current_goal].position.x - robot_pose.x) * 180
					/ M_PI);
		}
	}
	return yaw;
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

void GraphPathCalculator::dynamicReconfigureCallback(
		rrg_nbv_exploration::GraphConstructorConfig &config, uint32_t level) {
	_distance_factor = config.distance_factor;
	_traversability_factor = config.traversability_factor;
	_heading_factor = config.heading_factor;
	_radius_factor = config.radius_factor;
}

}
