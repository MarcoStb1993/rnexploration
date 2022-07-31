/*
 * AedeMessageInterface.cpp
 *
 *  Created on: Jul 31, 2022
 *      Author: marco
 */

#include <rrg_nbv_exploration_plugins/AedeMessageInterface.h>

namespace rne {
AedeMessageInterface::AedeMessageInterface() :
		_tf_listener(_tf_buffer) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame,

	"base_footprint");
	_cmd_vel_publisher = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1,
			true);
	_cmd_vel_stamped_subscriber = _nh.subscribe("cmd_vel_stamped", 1,
			&AedeMessageInterface::cmdVelStampedCallback, this);
	_velodyne_points_subscriber = _nh.subscribe("velodyne_points", 1,
			&AedeMessageInterface::velodynePointsCallback, this);
	_registered_velodyne_points_publisher = _nh.advertise<
			sensor_msgs::PointCloud2>("registered_velodyne_points", 1, true);

}

AedeMessageInterface::~AedeMessageInterface() {
	// TODO Auto-generated destructor stub
}

void AedeMessageInterface::cmdVelStampedCallback(
		const geometry_msgs::TwistStamped::ConstPtr &vel) {
	geometry_msgs::Twist cmd;
	cmd = vel->twist;
	_cmd_vel_publisher.publish(cmd);
}

void AedeMessageInterface::velodynePointsCallback(
		const sensor_msgs::PointCloud2::ConstPtr &point_cloud) {
	geometry_msgs::TransformStamped transformStamped;
	try {
		transformStamped = _tf_buffer.lookupTransform("map", _robot_frame,
				ros::Time(0));
		sensor_msgs::PointCloud2 registered_cloud;
		tf2::doTransform(*point_cloud, registered_cloud, transformStamped);
		_registered_velodyne_points_publisher.publish(registered_cloud);
	} catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
	}
}

}
