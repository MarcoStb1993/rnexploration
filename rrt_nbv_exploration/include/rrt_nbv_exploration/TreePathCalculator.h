#include "ros/ros.h"
#include <rrt_nbv_exploration_msgs/Tree.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rrt_nbv_exploration {
/**
 * @brief The TreePathCalculator class calculates a traversable path for the robot to a new node from it's parent node.
 */
class TreePathCalculator {
public:
	/**
	 * @brief Constructor that initializes the node handle, parameters and a publisher for raytracing visualization
	 */
	TreePathCalculator();

	/**
	 * @brief Returns the robot's pose in the map frame
	 * @return Robot pose
	 */
	geometry_msgs::Pose getRobotPose();

	/**
	 * @brief Returns distance from the robot's current position to the given node
	 * @param Position of the node
	 * @return Distance between current position and provided node in m
	 */
	double getDistanceToNode(geometry_msgs::Point node);

	/**
	 * @brief Calculates a path from the given start node to the goal node moving only along the tree's edges
	 * @param Reference to the calculated path
	 * @param Current tree
	 * @param Node to start from
	 * @param Node to go to
	 */
	void calculatePath(std::vector<geometry_msgs::PoseStamped> &path,
			rrt_nbv_exploration_msgs::Tree &rrt, int start_node, int goal_node);

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
