#include "ros/ros.h"
#include <rrt_nbv_exploration_msgs/Tree.h>
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
	 * @brief Initializes given nodes path from the robot's current position to it and its distance by taking the node's
	 * parent's path and adding the new node to it
	 * @param Reference to the node to be initialized
	 * @param Parent node's path to robot
	 * @param Parent node's distance to robot
	 */
	void initializePathToRobot(rrt_nbv_exploration_msgs::Node &node,
			std::vector<int> parentPathtoRobot, double parentDistanceToRobot);

	/**
	 * @brief Updates paths and distances from the robot's current position to the respective node if it is still unexplored
	 * @param Index of the previous node closest to the robot
	 * @param Index of the new node closest to the robot
	 * @param Current tree
	 */
	void updatePathsToRobot(int prevNode, int newNode,
			rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Recalculate paths and distances from the robot's current position to the respective node if it is still unexplored
	 * @param Index of the previous node closest to the robot
	 * @param Index of the new node closest to the robot
	 * @param Current tree
	 */
	void recalculatePathsToRobot(int prevNode, int newNode,
			rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Returns a path from the node closest to the robot to the goal node moving only along the tree's edges
	 * @param Reference to the calculated path
	 * @param Current tree
	 * @param Node to go to
	 */
	void getNavigationPath(std::vector<geometry_msgs::PoseStamped> &path,
			rrt_nbv_exploration_msgs::Tree &rrt, int goal_node);

	/**
	 * @brief Returns if the two given nodes are next to each other
	 * @param Current tree
	 * @param Node started from
	 * @param Node went to
	 * @return If nodes are neighbors
	 */
	bool neighbourNodes(rrt_nbv_exploration_msgs::Tree &rrt, int startNode,
			int endNode);

	/**
	 * @brief Returns a path from the start node to the goal node moving only along the tree's edges
	 * @param Node to start from
	 * @param Node to go to
	 * @param Current tree
	 * @return List of node indexes on the path
	 */
	std::vector<int> findConnectingPath(int startNode, int goalNode,
			rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Calculate the new distance for the given path depending on the edge length (if lesser equals 0, retrieve
	 * particular distances)
	 * @param Index of the previous node closest to the robot
	 * @param Index of the new node closest to the robot
	 * @param Current path length
	 * @param Add to (true) or subtract from (false) distance
	 * @param Current tree
	 * @return Path length in m
	 */
	double updatePathDistance(int prevNode, int newNode, double path_length,
			bool add, rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Calculate the distance of the given path depending on the edge length (if lesser equals 0, retrieve
	 * particular distances)
	 * @param Current tree
	 * @param Path to calculate distance from
	 * @return Path length in m
	 */
	double calculatePathDistance(rrt_nbv_exploration_msgs::Tree &rrt,
			std::vector<int> path);

private:
	ros::NodeHandle _nh;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;
	/**
	 * @brief Fixed length of the tree's edges, flexible if set to -1 (TODO: wavefront if set to 0)
	 */
	double _edge_length;

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
