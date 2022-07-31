/*
 * AedeMessageInterface.h
 *
 *  Created on: Jul 31, 2022
 *      Author: marco
 */

#ifndef RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEMESSAGEINTERFACE_H_
#define RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEMESSAGEINTERFACE_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>

namespace rne {

/**
 * Refactors messages of the Autonomous Exploration Development Environment (AEDE) which is used for
 * local planning including terrain analysis. The required point cloud for terrain analysis is
 * transformed from the sensor to the map frame and the command velocity is changed from a TwistStamped
 * to a Twist message.
 */
class AedeMessageInterface {
public:
	AedeMessageInterface();
	virtual ~AedeMessageInterface();
private:
	ros::NodeHandle _nh;
	ros::Subscriber _cmd_vel_stamped_subscriber;
	ros::Publisher _cmd_vel_publisher;
	ros::Subscriber _velodyne_points_subscriber;
	ros::Publisher _registered_velodyne_points_publisher;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;

	void cmdVelStampedCallback(
			const geometry_msgs::TwistStamped::ConstPtr &vel);
	void velodynePointsCallback(
			const sensor_msgs::PointCloud2::ConstPtr &point_cloud);
};
}
#endif /* RRG_NBV_EXPLORATION_PLUGINS_SRC_AEDEMESSAGEINTERFACE_H_ */
