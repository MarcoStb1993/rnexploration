#include "ros/ros.h"
#include <rrg_nbv_exploration_msgs/Graph.h>
#include <rrg_nbv_exploration_msgs/Node.h>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rrg_nbv_exploration {
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
	 * @brief Extract the yaw angle in degrees from the robot pose
	 * @param Robot pose
	 * @return Robot yaw angle in degrees [0,360)
	 */
	int getRobotYaw(const geometry_msgs::Pose &robot_pos);

	/**
	 * @brief Initializes given nodes path from the robot's current position to it and its distance by taking the node's
	 * parent's path and adding the new node to it
	 * @param Reference to the node to be initialized
	 * @param Parent node's path to robot
	 * @param Parent node's distance to robot
	 */
	void initializePathToRobot(rrg_nbv_exploration_msgs::Node &node,
			std::vector<int> parentPathtoRobot, double parentDistanceToRobot);

	/**
	 * @brief Updates paths and distances from the robot's current position to the respective nodes using Dijkstra's algorithm
	 * @param Index of the node from which Dijkstra's is started
	 * @param Current graph
	 * @param Robot pose
	 * @param If all current paths and distances should be reset or only a "local update" is necessary
	 */
	void updatePathsToRobot(int startNode, rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Pose robot_pos, bool reset = true);

	/**
	 * @brief Updates heading change from the robot's current position due to a heading change of the robot
	 * @param Index of the node closest to the robot
	 * @param Current graph
	 * @param Robot pose
	 * @return True if node headings were updated
	 */
	bool updateHeadingToRobot(int startNode, rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Pose robot_pos);

	/**
	 * @brief Returns a path from the node closest to the robot to the goal node moving only along the tree's edges
	 * @param Reference to the calculated path
	 * @param Current tree
	 * @param Node to go to
	 * @param Actual position of the robot
	 */
	void getNavigationPath(std::vector<geometry_msgs::PoseStamped> &path,
			rrg_nbv_exploration_msgs::Graph &rrg, int goal_node,
			geometry_msgs::Point robot_pose);

	/**
	 * @brief Returns if the two given nodes are next to each other by searching the the graphs edges
	 * @param Current graph
	 * @param Node started from
	 * @param Node went to
	 * @return If nodes are neighbors
	 */
	bool neighbourNodes(rrg_nbv_exploration_msgs::Graph &rrg, int startNode,
			int endNode);

	/**
	 * @brief Returns the absolute angle difference in degrees between two given angles in degrees
	 * @param First angle
	 * @param Second angle
	 * @return Absolute angle difference
	 */
	int getAbsoluteAngleDiff(int x, int y);

private:
	ros::NodeHandle _nh;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;
	/**
	 * @brief Previous yaw of the robot in degrees
	 */
	int _last_robot_yaw;

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

	/**
	 * @brief Update the given node's heading change to best view from robot and update the largest change
	 * @brief Node index
	 * @brief Reference to RRG
	 */
	void setHeadingChangeToBestView(int node,
			rrg_nbv_exploration_msgs::Graph &rrg);
};
}
