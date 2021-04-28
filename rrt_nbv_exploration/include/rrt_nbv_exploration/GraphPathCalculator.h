#include "ros/ros.h"
#include <rrt_nbv_exploration_msgs/Graph.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rrt_nbv_exploration {
/**
 * @brief The TreePathCalculator class calculates a traversable path for the robot to a new node from it's parent node.
 */
class GraphPathCalculator {
public:
	/**
	 * @brief Constructor that initializes the node handle, parameters and a publisher for raytracing visualization
	 */
	GraphPathCalculator();

	/**
	 * @brief Returns the robot's pose in the map frame
	 * @return Robot pose
	 */
	geometry_msgs::Pose getRobotPose();

	/**
	 * @brief Calculate the Euclidean distance to the robot for the given node
	 * @param Node
	 * @param Current robot position
	 */
	void calculateDistanceToRobot(rrt_nbv_exploration_msgs::Node &node,
			geometry_msgs::Point robot_pose);

	/**
	 * @brief Calculate the Euclidean distance to the robot for each node
	 * @param Current tree
	 * @param Current robot position
	 */
	void calculateDistancesToRobot(rrt_nbv_exploration_msgs::Graph &rrt,
			geometry_msgs::Point robot_pose);

private:
	ros::NodeHandle _nh;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;

};
}
