#include "rrt_nbv_exploration/CollisionChecker.h"

namespace rrt_nbv_exploration {
CollisionChecker::CollisionChecker() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param("min_extend_range", _min_extend_range, 2.0);
	private_nh.param("robot_height", _robot_height, 1.0);
	private_nh.param("robot_width", _robot_width, 1.0);
	private_nh.param("robot_radius", _robot_radius, 1.0);
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
	received_grid = false;
}

bool CollisionChecker::steer3D(rrt_nbv_exploration_msgs::Node &new_node,
		rrt_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample, double min_distance) {
	double distance = sqrt(min_distance);
	if (distance >= 2 * _robot_radius) {
		fcl::OcTree* tree = new fcl::OcTree(_octree);
		std::shared_ptr<fcl::CollisionGeometry> tree_obj = std::shared_ptr
				< fcl::CollisionGeometry > (tree);
		std::shared_ptr<fcl::CollisionGeometry> cylinder(
				new fcl::Cylinder(_robot_radius, _robot_height));

		fcl::Transform3f tf_tree, tf_start_cylinder, tf_goal_cylinder,
				tf_path_box;
		tf_start_cylinder.setTranslation(
				fcl::Vec3f(nearest_node.position.x, nearest_node.position.y,
						_robot_height / 2));
		tf_goal_cylinder.setTranslation(
				fcl::Vec3f(rand_sample.x, rand_sample.y, _robot_height / 2));

		fcl::CollisionObject tree_collision(tree_obj, tf_tree);
		fcl::CollisionObject start_cylinder_collision(cylinder,
				tf_start_cylinder);
		fcl::CollisionObject goal_cylinder_collision(cylinder,
				tf_goal_cylinder);

		fcl::CollisionResult result_start_cylinder, result_goal_cylinder,
				result_path_box;
		fcl::CollisionRequest request_start_cylinder, request_goal_cylinder,
				request_path_box;
		std::size_t collision_start_cylinder = fcl::collide(&tree_collision,
				&start_cylinder_collision, request_start_cylinder,
				result_start_cylinder);
		std::size_t collision_goal_cylinder = fcl::collide(&tree_collision,
				&goal_cylinder_collision, request_goal_cylinder,
				result_goal_cylinder);
		std::size_t collision_path_box;

		geometry_msgs::Point center;
		double yaw;

		if (distance > _path_box_distance_thres) {
			std::shared_ptr<fcl::CollisionGeometry> box(
					new fcl::Box(distance - _path_box_distance_thres,
							_robot_width, _robot_height));
			center.x = (nearest_node.position.x + rand_sample.x) / 2;
			center.y = (nearest_node.position.y + rand_sample.y) / 2;
			center.z = _robot_height / 2;
			tf_path_box.setTranslation(
					fcl::Vec3f(center.x, center.y, center.z));
			fcl::Matrix3f rot_path_box;
			yaw = atan2(rand_sample.y - nearest_node.position.y,
					rand_sample.x - nearest_node.position.x);
			rot_path_box.setEulerZYX(0, M_PI / 4, yaw);
			tf_path_box.setRotation(rot_path_box);
			fcl::CollisionObject path_box_collision(box, tf_path_box);
			collision_path_box = fcl::collide(&tree_collision,
					&path_box_collision, request_path_box, result_path_box);
		}

		if (_visualize_collision) {
			visualizeCollisionCheck(nearest_node.position, rand_sample, center,
					distance, yaw, collision_start_cylinder,
					collision_goal_cylinder, collision_path_box);
		}

		if (!collision_start_cylinder && !collision_goal_cylinder
				&& !collision_path_box) {
			new_node.position.x = rand_sample.x;
			new_node.position.y = rand_sample.y;
			new_node.position.z = rand_sample.z;
			new_node.children_counter = 0;
			new_node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
//			ROS_INFO_STREAM(
//					"Collision with capsule at " << tf_capsule.getTranslation()[0] << ",  " << tf_capsule.getTranslation()[1] << ",  " << tf_capsule.getTranslation()[2] << " detected: " << res << " distance is: " << distance);
			return true;
		}
	}
	return false;
}

bool CollisionChecker::isCircleInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, nav_msgs::OccupancyGrid &vis_map) {
	unsigned int map_x, map_y;
	if (!worldToMap(center_x, center_y, map_x, map_y, map))
		return true;
	unsigned int radius = (int) (_robot_radius / map.info.resolution);
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

bool CollisionChecker::isLineInCollision(int y_start, int y_end, int x,
		nav_msgs::OccupancyGrid &map, nav_msgs::OccupancyGrid &vis_map) {
//	ROS_INFO_STREAM(
//			"is line in collision? y-start: " << y_start << " y-end: " << y_end << " x: " << x);
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
	if (_visualize_collision) {
		vis_map.header.stamp = ros::Time::now();
		vis_map.info.map_load_time = ros::Time::now();
		if (!received_grid) {
			vis_map.header.frame_id = "/map";
			vis_map.info.resolution = map.info.resolution;
			vis_map.info.width = map.info.width;
			vis_map.info.height = map.info.height;
			vis_map.info.origin = map.info.origin;
			vis_map.data = std::vector<int8_t>(map.info.width * map.info.height,
					-1);
			received_grid = true;
		}
	}
	if (!isCircleInCollision(rand_sample.x, rand_sample.y, map, vis_map)) {
		new_node.position.x = rand_sample.x;
		new_node.position.y = rand_sample.y;
		new_node.position.z = rand_sample.z;
		new_node.children_counter = 0;
		new_node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
		if (_visualize_collision) {
			_visualization_pub.publish(vis_map);
		}
		return true;
	}
	if (_visualize_collision) {
		_visualization_pub.publish(vis_map);
	}
	return false;
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
	ROS_INFO_STREAM("received occupancy grid");
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
	ROS_INFO_STREAM(
			"calculate path from " << start_node << " to " << goal_node);
	if (start_node == goal_node) {
		geometry_msgs::PoseStamped path_pose;
		path_pose.header.frame_id = "map";
		path_pose.header.stamp = ros::Time::now();
		path_pose.pose.position = rrt.nodes[start_node].position;
		tf2::Quaternion quaternion;
		quaternion.setRPY(0, 0, M_PI * rrt.nodes[start_node].best_yaw / 180.0);
		quaternion.normalize();
		path_pose.pose.orientation = tf2::toMsg(quaternion);
		path.push_back(path_pose);
	} else {
		bool continue_start = (start_node == 0 ? false : true), continue_goal =
				(goal_node == 0 ? false : true);
		std::vector<int> start_path, goal_path;
		start_path.push_back(start_node);
		goal_path.push_back(goal_node);
		while (continue_start || continue_goal) {
			if (continue_start) {
				start_path.push_back(rrt.nodes[start_path.back()].parent);
				ROS_INFO_STREAM(start_path.back() << " added to start_path");
				if (start_path.back() == 0)
					continue_start = false; //root node added
				auto result = std::find(goal_path.begin(), goal_path.end(),
						start_path.back());
				if (result != goal_path.end()) {
					ROS_INFO_STREAM("Found in goal_path!");
					goal_path.erase(result, goal_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
			if (continue_goal) {
				goal_path.push_back(rrt.nodes[goal_path.back()].parent);
				ROS_INFO_STREAM(goal_path.back() << " added to goal_path");
				if (goal_path.back() == 0)
					continue_goal = false;	//root node added
				auto result = std::find(start_path.begin(), start_path.end(),
						goal_path.back());
				if (result != start_path.end()) {
					ROS_INFO_STREAM("Found in start_path!");
					start_path.erase(result, start_path.end());
					continue_start = false;
					continue_goal = false;
				}
			}
		}
		start_path.insert(start_path.end(), goal_path.rbegin(),
				goal_path.rend());
		ROS_INFO_STREAM("Nodes in path:");
		for (auto &i : start_path) {
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.frame_id = "map";
			path_pose.header.stamp = ros::Time::now();
			path_pose.pose.position = rrt.nodes[i].position;
			tf2::Quaternion quaternion;
			quaternion.setRPY(0, 0, M_PI * rrt.nodes[i].best_yaw / 180.0);
			quaternion.normalize();
			path_pose.pose.orientation = tf2::toMsg(quaternion);
			path.push_back(path_pose);
			ROS_INFO_STREAM(
					i << ": x: " << path_pose.pose.position.x << ", y: " << path_pose.pose.position.y);
		}
	}
}

}
