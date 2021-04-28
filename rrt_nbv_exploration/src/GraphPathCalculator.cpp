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

void GraphPathCalculator::calculateDistanceToRobot(
		rrt_nbv_exploration_msgs::Node &node, geometry_msgs::Point robot_pose) {
	node.distanceToRobot = sqrt(
			pow((node.position.x - robot_pose.x), 2)
					+ pow((node.position.y - robot_pose.y), 2));
}

void GraphPathCalculator::calculateDistancesToRobot(
		rrt_nbv_exploration_msgs::Graph &rrg, geometry_msgs::Point robot_pose) {
	for (rrt_nbv_exploration_msgs::Node &node : rrg.nodes) {
		calculateDistanceToRobot(node, robot_pose);
	}
}



}
