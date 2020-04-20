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
	std::string octomap_topic;
	private_nh.param<std::string>("octomap_collision_topic",octomap_topic, "octomap_binary");
	private_nh.param<std::string>("robot_frame", _robot_frame, "base_link");
	ros::NodeHandle nh("rne");
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&CollisionChecker::convertOctomapMsgToOctree, this);
	if (_visualize_collision) {
		_collision_visualization = nh.advertise<visualization_msgs::Marker>(
				"steering_visualization", 10);
	}
	_path_box_distance_thres = 2
			* sqrt(pow(_robot_radius, 2) - pow(_robot_width / 2, 2));
}

bool CollisionChecker::steer(rrt_nbv_exploration_msgs::Node &new_node,
		rrt_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample, double min_distance) {
	double distance = sqrt(min_distance);
	if (distance >= _min_extend_range) {
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

		if (_collision_visualization) {
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

double CollisionChecker::getDistanceToNode(geometry_msgs::Point node){
	geometry_msgs::Point robot = getRobotPose().position;
	return sqrt(pow(robot.x - node.x,2) + pow(robot.y - node.y,2) + pow(robot.z-node.z,2));
}

void CollisionChecker::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr& map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast < octomap::OcTree > (_abstract_octree);
}

}
