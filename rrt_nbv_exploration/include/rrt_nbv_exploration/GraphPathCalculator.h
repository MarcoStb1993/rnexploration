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
	 * @brief Initializes given nodes path from the robot's current position to it and its distance by taking the node's
	 * parent's path and adding the new node to it
	 * @param Reference to the node to be initialized
	 * @param Parent node's path to robot
	 * @param Parent node's distance to robot
	 */
	void initializePathToRobot(rrt_nbv_exploration_msgs::Node &node,
			std::vector<int> parentPathtoRobot, double parentDistanceToRobot);

	/**
	 * @brief Updates paths and distances from the robot's current position to the respective nodes using Dijkstra's algorithm
	 * @param Index of the new node closest to the robot
	 * @param Current graph
	 * @param If all current paths and distances should be reset or only a "local update" is necessary
	 */
	void updatePathsToRobot(int startNode,
			rrt_nbv_exploration_msgs::Graph &rrg, bool reset = true);

	/**
	 * @brief Returns a path from the node closest to the robot to the goal node moving only along the tree's edges
	 * @param Reference to the calculated path
	 * @param Current tree
	 * @param Node to go to
	 * @param Actual position of the robot
	 */
	void getNavigationPath(std::vector<geometry_msgs::PoseStamped> &path,
			rrt_nbv_exploration_msgs::Graph &rrg, int goal_node, geometry_msgs::Point robot_pose);

	/**
	 * @brief Returns if the two given nodes are next to each other by searching the the graphs edges
	 * @param Current graph
	 * @param Node started from
	 * @param Node went to
	 * @return If nodes are neighbors
	 */
	bool neighbourNodes(rrt_nbv_exploration_msgs::Graph &rrg, int startNode,
			int endNode);

private:
	ros::NodeHandle _nh;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;

	/**
	 * @brief Add a path node every 10cm in between the start and end point along the given path
	 * @param Reference to the calculated path
	 * @param Starting position
	 * @param End position
	 * @param Orientation between the nodes
	 * @param Yaw between the nodes
	 */
	void addInterNodes(std::vector<geometry_msgs::PoseStamped> &path,
			geometry_msgs::Point start, geometry_msgs::Point end,
			geometry_msgs::Quaternion orientation, double yaw);
};
}
