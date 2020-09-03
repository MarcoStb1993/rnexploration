#include "rrt_nbv_exploration/CollisionChecker.h"

namespace rrt_nbv_exploration {
CollisionChecker::CollisionChecker() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param("min_extend_range", _min_extend_range, 2.0);
	private_nh.param("robot_height", _robot_height, 1.0);
	private_nh.param("robot_width", _robot_width, 1.0);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	private_nh.param("visualize_collision", _visualize_collision, false);
	std::string octomap_topic, occupancy_grid_topic;
	private_nh.param<std::string>("octomap_collision_topic", octomap_topic,
			"octomap_binary");
	private_nh.param<std::string>("occupancy_grid_topic", occupancy_grid_topic,
			"map");
	private_nh.param<std::string>("robot_frame", _robot_frame, "base_link");
	ros::NodeHandle nh("rne");
//	_octomap_sub = _nh.subscribe(octomap_topic, 1,
//			&CollisionChecker::convertOctomapMsgToOctree, this);
	_occupancy_grid_sub = _nh.subscribe(occupancy_grid_topic, 1,
			&CollisionChecker::occupancyGridCallback, this);
	if (_visualize_collision) {
//		_collision_visualization = nh.advertise<visualization_msgs::Marker>(
//				"steering_visualization", 10);
		_visualization_pub = nh.advertise<nav_msgs::OccupancyGrid>(
				"collision_visualization", 1);
	}
	_path_box_distance_thres = 2
			* sqrt(pow(_robot_radius, 2) - pow(_robot_width / 2, 2));
	_init_vis_map = false;
}

bool CollisionChecker::isCircleInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, nav_msgs::OccupancyGrid &vis_map) {
	unsigned int map_x, map_y;
	if (!worldToMap(center_x, center_y, map_x, map_y, map))
		return true;
	unsigned int radius = (int) (_robot_radius / map.info.resolution);
//	ROS_INFO_STREAM(
//			"Circle with center " << center_x << ", " << center_y << ": " << map_x << ", " << map_y << ", radius: " << radius);
	int x = radius, y = 0, err = 0;
	bool collision = false;
	while (x >= y) {
		if (isLineInCollision(map_x - x, map_x + x, map_y + y, map, vis_map))
			collision = true; //return true;
		if (isLineInCollision(map_x - x, map_x + x, map_y - y, map, vis_map))
			collision = true; //return true;
		if (isLineInCollision(map_x - y, map_x + y, map_y + x, map, vis_map))
			collision = true; //return true;
		if (isLineInCollision(map_x - y, map_x + y, map_y - x, map, vis_map))
			collision = true; //return true;
		if (err <= 0) {
			y += 1;
			err += 2 * y + 1;
		} else {
			x -= 1;
			err -= 2 * x + 1;
		}
	}
	return collision; //false;
}

bool CollisionChecker::isRectangleInCollision(double x, double y, double yaw,
		double half_height, double half_width, nav_msgs::OccupancyGrid &map,
		nav_msgs::OccupancyGrid &vis_map) {
	std::array<point, 4> corners;
	double cos_yaw, sin_yaw;
	bool collision = false;
	cos_yaw = cos(yaw);
	sin_yaw = sin(yaw);
//	ROS_INFO_STREAM(
//			std::setprecision(3) << std::fixed << "Square with center " << x << ", " << y << ": yaw: " << yaw* (180.0/3.141592653589793238463) <<", 0:" << x - half_height * cos_yaw + half_width * sin_yaw << ", " << y - half_height * sin_yaw - half_width * cos_yaw << ", 1: " << x + half_height * cos_yaw + half_width * sin_yaw << ", " << y + half_height * sin_yaw - half_width * cos_yaw << ", 2: " << x + half_height * cos_yaw - half_width * sin_yaw << ", " << y + half_height * sin_yaw + half_width * cos_yaw << ", 3: " << x - half_height * cos_yaw - half_width * sin_yaw << ", " << y - half_height * sin_yaw + half_width * cos_yaw);
	if (!worldToMap(x - half_height * cos_yaw - half_width * sin_yaw,
			y - half_height * sin_yaw + half_width * cos_yaw, corners[3].x,
			corners[3].y, map)
			|| !worldToMap(x + half_height * cos_yaw - half_width * sin_yaw,
					y + half_height * sin_yaw + half_width * cos_yaw,
					corners[2].x, corners[2].y, map)
			|| !worldToMap(x + half_height * cos_yaw + half_width * sin_yaw,
					y + half_height * sin_yaw - half_width * cos_yaw,
					corners[1].x, corners[1].y, map)
			|| !worldToMap(x - half_height * cos_yaw + half_width * sin_yaw,
					y - half_height * sin_yaw - half_width * cos_yaw,
					corners[0].x, corners[0].y, map))
		return true;
//	ROS_INFO_STREAM(
//			"Square with center " << x << ", " << y << ": 0:" << corners[0].x << ", " << corners[0].y << ", 1: " << corners[1].x << ", " << corners[1].y << ", 2: " << corners[2].x << ", " << corners[2].y << ", 3: " << corners[3].x << ", " << corners[3].y);
	if ((corners[0].x == corners[1].x && corners[1].y == corners[2].y)
			|| (corners[1].x == corners[2].x && corners[2].y == corners[3].y)) {
//		ROS_INFO("aligned");
//		ROS_INFO_STREAM(
//				"Square with center " << x << ", " << y << ": 0:" << corners[0].x << ", " << corners[0].y << ", 1: " << corners[1].x << ", " << corners[1].y << ", 2: " << corners[2].x << ", " << corners[2].y << ", 3: " << corners[3].x << ", " << corners[3].y);
		bool y0_top = corners[0].y > corners[2].y;
		bool x0_left = corners[0].x < corners[2].x;
//		ROS_INFO_STREAM("Top=x0: " << y0_top << " Left=y0: " << x0_left);
		for (unsigned int i = (y0_top ? corners[2].y : corners[0].y);
				i <= (y0_top ? corners[0].y : corners[2].y); i++) {
//			ROS_INFO_STREAM("line at x: " << i << " from " << (x0_left ? corners[0].y : corners[2].y) << " to " << (x0_left ? corners[2].y : corners[0].y));
			if (isLineInCollision(x0_left ? corners[0].x : corners[2].x,
					x0_left ? corners[2].x : corners[0].x, i, map, vis_map))
				collision = true; //return true;
		}
	} else {
		//Mid point
		auto it_bot =
				std::min_element(std::begin(corners), std::end(corners),
						[](const point &p1, const point &p2) {return p1.y < p2.y;});
		std::size_t bot_corner = std::distance(std::begin(corners), it_bot);
		auto it_left =
				std::min_element(std::begin(corners), std::end(corners),
						[](const point &p1, const point &p2) {return p1.x < p2.x;});
		std::size_t left_corner = std::distance(std::begin(corners), it_left);
		std::size_t right_corner = (left_corner + 2) % 4, top_corner =
				(bot_corner + 2) % 4;
//		ROS_INFO_STREAM(
//				"Bot: " << bot_corner << " left: " << left_corner<< " right: " << right_corner << " top: " << top_corner);
		float l_m = ((float) corners[left_corner].x
				- (float) corners[bot_corner].x)
				/ ((float) corners[left_corner].y
						- (float) corners[bot_corner].y);
		float r_m = ((float) corners[right_corner].x
				- (float) corners[bot_corner].x)
				/ ((float) corners[right_corner].y
						- (float) corners[bot_corner].y);
//		ROS_INFO_STREAM("l_m: " << l_m << " r_m: " <<r_m);
		unsigned int lr_y = corners[bot_corner].y;
		float l_x = (float) corners[bot_corner].x, r_x = l_x;
		if (isLineInCollision(l_x, r_x, lr_y, map, vis_map))
			collision = true; //return true;
		while (lr_y < corners[top_corner].y) {
			if (lr_y == corners[left_corner].y) {
				l_m = ((float) corners[top_corner].x
						- (float) corners[left_corner].x)
						/ ((float) corners[top_corner].y
								- (float) corners[left_corner].y);
				l_x = (float) corners[left_corner].x;
//				ROS_INFO_STREAM("new l_m: " << l_m);
			}
			if (lr_y == corners[right_corner].y) {
				r_m = ((float) corners[top_corner].x
						- (float) corners[right_corner].x)
						/ ((float) corners[top_corner].y
								- (float) corners[right_corner].y);
				r_x = (float) corners[right_corner].x;
//				ROS_INFO_STREAM("new r_m: " <<r_m);
			}
			l_x += l_m;
			r_x += r_m;
			lr_y++;
//			ROS_INFO_STREAM(
//					"line x: " << lr_y << " l_x: " << l_x << " r_x: " << r_x);
//			ROS_INFO_STREAM(
//					"line(rounded) x: " << lr_y << " l_x: " << (unsigned int) round(l_x) << " r_x: " << (unsigned int) round(r_y));
			if (isLineInCollision((unsigned int) round(l_x),
					(unsigned int) round(r_x), lr_y, map, vis_map))
				collision = true; //return true;
		}

	}
	return collision;
}

bool CollisionChecker::isLineInCollision(int y_start, int y_end, int x,
		nav_msgs::OccupancyGrid &map, nav_msgs::OccupancyGrid &vis_map) {
//	ROS_INFO_STREAM(
//			"is line in collision? x: " << x <<" y-start: " << y_start << " y-end: " << y_end);
//	ROS_INFO_STREAM(
//			"map width: " << map.info.width << " map height: " << map.info.width << " map data length: " << map.data.size() << "map at 0 " << (int)map.data[0]);
	if (y_start < 0 || y_end > map.info.width || x < 0 || x > map.info.height)
		return true;
	bool collision = false;
	for (int y = x * map.info.width + y_start; y <= x * map.info.width + y_end;
			y++) {
		if (map.data[y] == -1 || map.data[y] >= 100) {
			collision = true; //return true;
			vis_map.data[y] = 100;
		} else {
			vis_map.data[y] = 0;
		}
	}
	return collision; //false;
}

bool CollisionChecker::steer(rrt_nbv_exploration_msgs::Node &new_node,
		rrt_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample, double min_distance) {
	nav_msgs::OccupancyGrid map = _occupancy_grid;
	double distance = sqrt(min_distance);
	//if (distance >= 2 * _robot_radius) {
		bool no_collision = false;
		if (_visualize_collision) {
			vis_map.header.stamp = ros::Time::now();
			vis_map.info.map_load_time = ros::Time::now();
			if (!_init_vis_map) {
				vis_map.header.frame_id = "/map";
				vis_map.info.resolution = map.info.resolution;
				vis_map.info.width = map.info.width;
				vis_map.info.height = map.info.height;
				vis_map.info.origin = map.info.origin;
				vis_map.data = std::vector<int8_t>(
						map.info.width * map.info.height, -1);
				_init_vis_map = true;
			}
		}
//	ROS_INFO_STREAM(
//			"Dist: "<< distance << " thres:" << _path_box_distance_thres << " rand x: " << rand_sample.x << " rand y: " << rand_sample.y);
//	ROS_INFO_STREAM("yaw: " << atan2(rand_sample.y - nearest_node.position.y,
//									rand_sample.x - nearest_node.position.x) << " rand: x: " << rand_sample.x << " ,y: " << rand_sample.y
//			<< " nearest: x: " << nearest_node.position.x << " ,y: " << nearest_node.position.y);
		bool rectangle = (
				distance > _path_box_distance_thres ?
						!isRectangleInCollision(
								(nearest_node.position.x + rand_sample.x) / 2,
								(nearest_node.position.y + rand_sample.y) / 2,
								atan2(rand_sample.y - nearest_node.position.y,
										rand_sample.x
												- nearest_node.position.x),
								(distance - _path_box_distance_thres) / 2,
								_robot_width / 2, map, vis_map) :
						true);
		bool circle = !isCircleInCollision(rand_sample.x, rand_sample.y, map,
				vis_map);
		if (rectangle && circle) {
			new_node.position.x = rand_sample.x;
			new_node.position.y = rand_sample.y;
			new_node.position.z = rand_sample.z;
			new_node.children_counter = 0;
			new_node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
			no_collision = true;
		}
		if (_visualize_collision) {
			_visualization_pub.publish(vis_map);
		}
		return no_collision;
//	}
//	return false;
}

void CollisionChecker::visualizeCollisionCheck(geometry_msgs::Point start,
		geometry_msgs::Point goal, geometry_msgs::Point center, double distance,
		double yaw, bool collision_start, bool collision_goal,
		bool collision_path) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.ns = "steering_visualization";
	marker.id = 0;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.color.a = 0.6f;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.header.stamp = ros::Time::now();
	marker.pose.position.x = start.x;
	marker.pose.position.y = start.y;
	marker.pose.position.z = _robot_height / 2;
	marker.scale.x = 2 * _robot_radius;
	marker.scale.y = 2 * _robot_radius;
	marker.scale.z = _robot_height;
	if (collision_start) {
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
	} else {
		marker.color.g = 1.0f;
		marker.color.r = 0.0f;
	}
	_collision_visualization.publish(marker);
	marker.id = 1;
	marker.pose.position.x = goal.x;
	marker.pose.position.y = goal.y;
	marker.pose.position.z = _robot_height / 2;
	if (collision_goal) {
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
	} else {
		marker.color.g = 1.0f;
		marker.color.r = 0.0f;
	}
	_collision_visualization.publish(marker);
	if (distance > _path_box_distance_thres) {
		marker.id = 2;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.pose.position.x = center.x;
		marker.pose.position.y = center.y;
		marker.pose.position.z = center.z;
		tf2::Quaternion quat_tf;
		quat_tf.setRPY(0, 0, yaw);
		marker.pose.orientation = tf2::toMsg(quat_tf);
		marker.scale.x = distance - _path_box_distance_thres;
		marker.scale.y = _robot_width;
		marker.scale.z = _robot_height;
		if (collision_path) {
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
		} else {
			marker.color.g = 1.0f;
			marker.color.r = 0.0f;
		}
		_collision_visualization.publish(marker);
	}
}

geometry_msgs::Pose CollisionChecker::getRobotPose() {
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

double CollisionChecker::getDistanceToNode(geometry_msgs::Point node) {
	geometry_msgs::Point robot = getRobotPose().position;
	return sqrt(
			pow(robot.x - node.x, 2) + pow(robot.y - node.y, 2)
					+ pow(robot.z - node.z, 2));
}

void CollisionChecker::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr& map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast < octomap::OcTree > (_abstract_octree);
}

void CollisionChecker::occupancyGridCallback(
		const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
//	ROS_INFO_STREAM("received occupancy grid");
	_occupancy_grid = *map_msg;
//	ROS_INFO_STREAM(
//			"Res: " << map_msg->info.resolution << " Width: " << map_msg->info.width<< " Height: " << map_msg->info.height << " x: "<<map_msg->info.origin.position.x << " y: " << map_msg->info.origin.position.y);
//	unsigned int mx, my;
//	if (worldToMap(0.0, 0.0, mx, my))
//		ROS_INFO_STREAM("Center x: " << mx << " y: " << my);
}

bool CollisionChecker::worldToMap(double wx, double wy, unsigned int& mx,
		unsigned int& my, nav_msgs::OccupancyGrid &map) {
	if (wx < map.info.origin.position.x || wy < map.info.origin.position.y)
		return false;
	mx = (int) ((wx - map.info.origin.position.x) / map.info.resolution);
	my = (int) ((wy - map.info.origin.position.y) / map.info.resolution);
	if (mx < map.info.height && my < map.info.width)
		return true;
	return false;
}

void CollisionChecker::calculatePath(
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
