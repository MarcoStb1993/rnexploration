#include "rrt_nbv_exploration/CollisionChecker.h"

namespace rrt_nbv_exploration {
CollisionChecker::CollisionChecker() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param("min_extend_range", _min_extend_range, 2.0);
	private_nh.param("visualize_steering", _visualize_steering, false);
	private_nh.param<std::string>("robot_frame", _robot_frame, "base_link");
	ros::NodeHandle nh("rne");
	_steering_visualization = nh.advertise<visualization_msgs::Marker>(
			"steering_visualization", 10);
	_octomap_sub = _nh.subscribe("octomap_binary_wo_ground", 1,
			&CollisionChecker::convertOctomapMsgToOctree, this);
}

bool CollisionChecker::steer(rrt_nbv_exploration_msgs::Node &new_node,
		rrt_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample, double min_distance) {
	if (min_distance >= _min_extend_range) {
		visualization_msgs::Marker _node_points;
		_node_points.header.frame_id = "/map";
		_node_points.ns = "steering_visualization";
		_node_points.id = 0;
		_node_points.action = visualization_msgs::Marker::ADD;
		_node_points.pose.orientation.w = 1.0;
		_node_points.type = visualization_msgs::Marker::SPHERE_LIST;
		_node_points.color.a = 1.0f;
		_node_points.header.stamp = ros::Time::now();

		fcl::OcTree* tree = new fcl::OcTree(_octree);
		std::shared_ptr<fcl::CollisionGeometry> tree_obj = std::shared_ptr
				< fcl::CollisionGeometry > (tree);
		double radius = 0.5;
		double length = min_distance;
		//std::shared_ptr<fcl::CollisionGeometry> sphere(new fcl::Sphere(radius));
		std::shared_ptr<fcl::CollisionGeometry> capsule(
				new fcl::Capsule(radius, length));
		fcl::Transform3f tf_tree, tf_sphere, tf_capsule;
		//tf_sphere.setTranslation(fcl::Vec3f(0,0, 0));
		tf_capsule.setTranslation(
				fcl::Vec3f((nearest_node.position.x + rand_sample.x) / 2,
						(nearest_node.position.y + rand_sample.y) / 2, 0));
		fcl::Matrix3f rot_capsule;
		double yaw = atan2(rand_sample.y - nearest_node.position.y,
				rand_sample.x - nearest_node.position.x);
		rot_capsule.setEulerZYX(0, M_PI / 4, yaw);
		tf_capsule.setRotation(rot_capsule);
		fcl::CollisionObject tree_collision(tree_obj, tf_tree);
		//fcl::CollisionObject sphere_collision(sphere, tf_sphere);
		fcl::CollisionObject capsule_collision(capsule, tf_capsule);
		fcl::CollisionResult result;
		fcl::CollisionRequest request;
		std::size_t res = fcl::collide(&tree_collision, &capsule_collision,
				request, result);

//		fcl::DistanceResult dresult;
//		dresult.clear();
//		fcl::DistanceRequest drequest(true);
//		double distance = fcl::distance(&tree_collision, &capsule_collision,
//				drequest, dresult);

		geometry_msgs::Point point;
		point.x = nearest_node.position.x;
		point.y = nearest_node.position.y;
		point.z = nearest_node.position.z;
		_node_points.scale.x = 2 * radius;
		_node_points.scale.y = 2 * radius;
		_node_points.scale.z = 2 * radius;
		std_msgs::ColorRGBA color;
		color.r = 0.0f;
		color.g = 0.0f;
		color.b = 1.0f;
		color.a = 1.0f;
		_node_points.points.push_back(point);
		_node_points.points.push_back(point);
		point.x = rand_sample.x;
		point.y = rand_sample.y;
		point.z = rand_sample.z;
		_node_points.scale.x = 2 * radius;
		_node_points.scale.y = 2 * radius;
		_node_points.scale.z = 2 * radius;
		color.r = 0.0f;
		color.g = 0.0f;
		color.b = 1.0f;
		color.a = 1.0f;
		_node_points.points.push_back(point);
		_node_points.points.push_back(point);

		new_node.position.x = rand_sample.x;
		new_node.position.y = rand_sample.y;
		new_node.position.z = rand_sample.z;
		new_node.children_counter = 0;
		new_node.status = rrt_nbv_exploration_msgs::Node::INITIAL;

		if (!res) {
//			ROS_INFO_STREAM(
//					"Collision with capsule at " << tf_capsule.getTranslation()[0] << ",  " << tf_capsule.getTranslation()[1] << ",  " << tf_capsule.getTranslation()[2] << " detected: " << res << " distance is: " << distance);
			_steering_visualization.publish(_node_points);
			return true;
		}
	}
	return false;
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

void CollisionChecker::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr& map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast < octomap::OcTree > (_abstract_octree);
}

}
