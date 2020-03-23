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
			"steering_visualization", 1000);
}

bool CollisionChecker::steer(rrt_nbv_exploration_msgs::Node &new_node,
		rrt_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample, double min_distance,
		boost::shared_ptr<octomap::OcTree> octree) {
	visualization_msgs::Marker _node_points;
	_node_points.header.frame_id = "/map";
	_node_points.ns = "steering_visualization";
	_node_points.id = 0;
	_node_points.action = visualization_msgs::Marker::ADD;
	_node_points.pose.orientation.w = 1.0;
	_node_points.type = visualization_msgs::Marker::SPHERE_LIST;
	_node_points.scale.x = 0.1;
	_node_points.scale.y = 0.1;
	_node_points.scale.z = 0.1;
	_node_points.color.a = 1.0f;
	_node_points.header.stamp = ros::Time::now();

	min_distance = sqrt(min_distance);
	if (min_distance >= _min_extend_range) {
		double extension =
				min_distance > _min_extend_range ?
						min_distance : _min_extend_range;
		octomap::point3d start_point(nearest_node.position.x,
				nearest_node.position.y, nearest_node.position.z);
		octomap::point3d end_point(
				nearest_node.position.x
						+ extension * (rand_sample.x - nearest_node.position.x)
								/ min_distance,
				nearest_node.position.y
						+ extension * (rand_sample.y - nearest_node.position.y)
								/ min_distance,
				nearest_node.position.z
						+ extension * (rand_sample.z - nearest_node.position.z)
								/ min_distance);
		geometry_msgs::Point last_coords;
		last_coords.x = start_point.x();
		last_coords.y = start_point.y();
		last_coords.z = start_point.z();
		bool blocked = false;
		octomap::KeyRay keyray;
		if (octree->computeRayKeys(start_point, end_point, keyray)) {
			for (auto iterator : keyray) {
				octomap::point3d coords = octree->keyToCoord(iterator);
				octomap::OcTreeNode* ocnode = octree->search(iterator);
				geometry_msgs::Point point;
				point.x = coords.x();
				point.y = coords.y();
				point.z = coords.z();
				std_msgs::ColorRGBA color;
				color.r = 0.0f;
				color.g = 0.0f;
				color.b = 1.0f;
				color.a = 1.0f;
				_node_points.points.push_back(point);
				if (ocnode != NULL) {
					bool occupied = octree->isNodeOccupied(ocnode);
					if (occupied) {
						color.r = 1.0f;
						color.b = 0.0f;
						blocked = true;

					} else {
						color.g = 1.0f;
						color.b = 0.0f;
					}
				}
				_node_points.colors.push_back(color);
				if (!blocked) {
					last_coords.x = coords.x();
					last_coords.y = coords.y();
					last_coords.z = coords.z();
				}

			}
		}
		if (blocked) {
			float distance = pow((last_coords.x - nearest_node.position.x), 2)
					+ pow((last_coords.y - nearest_node.position.y), 2)
					+ pow((last_coords.z - nearest_node.position.z), 2);
			if (distance >= _min_extend_range * _min_extend_range) {
				new_node.position.x = last_coords.x;
				new_node.position.y = last_coords.y;
				new_node.position.z = last_coords.z;
				new_node.children_counter = 0;
				new_node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
				if (_visualize_steering) {
					_steering_visualization.publish(_node_points);
				}
				return true;
			}
		} else {
			new_node.position.x = last_coords.x;
			new_node.position.y = last_coords.y;
			new_node.position.z = last_coords.z;
			new_node.children_counter = 0;
			new_node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
			if (_visualize_steering) {
				_steering_visualization.publish(_node_points);
			}
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

}
