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
#include <rrg_nbv_exploration/GraphConstructorConfig.h>

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
	 * @brief Updates paths and costs from the given start node to the respective nodes using
	 * Dijkstra's algorithm
	 * @param Index of the node from which Dijkstra's is started
	 * @param Current graph
	 * @param Robot pose
	 * @param If all current paths and distances should be reset or only a "local update" is necessary
	 * @param Reference to list of nodes indices which should be updated, newly reachable nodes will be
	 * added to this list
	 * @param Reference to if a node was added to the list of nodes to update
	 */
	void updatePathsToRobot(int startNode, rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Pose robot_pos, bool reset,
			std::list<int> &nodes_to_update, bool &added_node_to_update);

	/**
	 * @brief Updates paths and costs from the robot's current position due to a heading change of the robot
	 * to all respective nodes using Dijkstra's algorithm
	 * @param Index of the node closest to the robot
	 * @param Current graph
	 * @param Reference to robot pose
	 * @param Index of the next node on the path to the current goal from the currently nearest node
	 * @param Index of the edge from the currently nearest node to the next node
	 * @param Squared distance to the currently nearest node in m
	 * @param Reference to list of nodes indices which should be updated, newly reachable nodes will be
	 * added to this list
	 * @param Reference to if a node was added to the list of nodes to update
	 * @return True if node headings were updated
	 */
	bool updateHeadingToRobot(int startNode,
			rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Pose &robot_pos, int next_node,
			int edge_to_next_node, double distance_to_nearest_node_squared,
			std::list<int> &nodes_to_update, bool &added_node_to_update);

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
	 * @brief Returns if the two given nodes are next to each other by searching the graphs edges
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

	/**
	 * @brief Determines the yaw for the current goal depending on the horizontal FoV and the path to it
	 * @param Index of the current goal node
	 * @param Reference to the RRG
	 * @param Actual position of the robot
	 * @param If the robot is currently going back to the root node because exploration finished (defaults to false)
	 * @return Heading in degrees for the current goal
	 */
	int determineGoalYaw(int current_goal, rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Point robot_pose, bool homing = false);

	/**
	 * @brief Calculate the cost function of the given node based on its distance, heading change,
	 * traversability and node radii to the robot
	 * @param Node to calculate cost function for
	 * @return Cost function
	 */
	double calculateCostFunction(rrg_nbv_exploration_msgs::Node &node);

	/**
	 * @brief Update the given node's heading change to best view from robot
	 * @param Reference to node which heading will be updated
	 * @return Heading change to best view in degrees
	 */
	int calculateHeadingChangeToBestView(rrg_nbv_exploration_msgs::Node &node);

	/**
	 * @brief Checks if an edge to the given neighbor node from the provided node already exists and
	 * returns its index if it does
	 * @param Reference to the RRG
	 * @param Index of the node
	 * @param Index of the neighbor node
	 * @return Index of the edge if it exists, -1 otherwise
	 */
	int findExistingEdge(rrg_nbv_exploration_msgs::Graph &rrg, int node,
			int neighbor_node);

	void dynamicReconfigureCallback(
			rrg_nbv_exploration::GraphConstructorConfig &config,
			uint32_t level);

private:
	ros::NodeHandle _nh;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;
	/**
	 * Radius that includes robot's footprint in m
	 */
	double _robot_radius;
	/**
	 * Squared radius that includes robot's footprint in m
	 */
	double _robot_radius_squared;
	/**
	 * Sensor's horizontal FoV that is considered for gain calculation in degrees
	 */
	int _sensor_horizontal_fov;
	/**
	 * @brief Previous yaw of the robot in degrees
	 */
	int _last_robot_yaw;
	/**
	 * @brief Weighting factor for the radius of a node (only active when inflation is active)
	 */
	double _radius_factor;
	/**
	 * @brief Weighting factor for the distance to a node
	 */
	double _distance_factor;
	/**
	 * @brief Weighting factor for the heading change while moving to a node
	 */
	double _heading_factor;
	/**
	 * @brief Weighting factor for the traversability cost along the path to a node
	 */
	double _traversability_factor;
	/**
	 * @brief If the inflation of nodes (wavefront) is active
	 */
	bool _inflation_active;
	/**
	 * @brief Grid map value that indicates a cell is occupied (or inscribed)
	 */
	int _grid_map_occupied;
	/**
	 * @brief Index of the node that was previously the nearest node to the robot
	 */
	int _last_nearest_node;

	/**
	 * @brief Find the yaw in deg and distance in m from the start node to the end node and return true if it was found
	 * @param Index of the start node
	 * @param Index of the end node
	 * @param Reference to the yaw to be set in deg
	 * @param Reference to the distance to be set in m
	 * @param Reference to the RRG
	 * @return If an edge between both nodes was found
	 */
	bool getHeadingBetweenNodes(int start_node, int end_node, int &yaw,
			double &distance, rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Add a path node every 10cm in between the start and end point along the given path
	 * @param Reference to the calculated path
	 * @param Starting position
	 * @param End position
	 * @param Orientation between the nodes
	 * @param Yaw between the nodes in rad
	 * @param Distance between the nodes in m
	 */
	void addInterNodes(std::vector<geometry_msgs::PoseStamped> &path,
			geometry_msgs::Point start, geometry_msgs::Point end,
			geometry_msgs::Quaternion orientation, double yaw, double distance);

	/**
	 * @brief Calculate a straight path from the robot to the given node
	 * @param Current robot position
	 * @param Goal node for the path
	 * @param Reference to the RRG
	 * @param Reference to the path to be calculated
	 */
	void getPathFromRobotToNode(geometry_msgs::Point robot_pose, int goal_node,
			rrg_nbv_exploration_msgs::Graph &rrg,
			std::vector<geometry_msgs::PoseStamped> &path);

	/**
	 * @brief Calculate the cost function for the provided node if it would be connected to the
	 * neighbor node via the given edge, return the modified node
	 * @param Index of the neighbor node from which the connection will be evaluated
	 * @param Index of the node which potential cost function will be calculated
	 * @param Index of the edge that connects both nodes
	 * @param Reference to the RRG
	 * @return Modified node with newly calculated cost function (and heading, traversability and distance)
	 */
	rrg_nbv_exploration_msgs::Node calculateCostFunctionForConnection(
			int neighbor_node, int node, int edge,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Implementation of Dijkstra's algorithm to find the path to each node with the lowest cost,
	 * starting with the nodes already in the given queue
	 * @param Queue of node indices for which neighbors the path cost must be checked
	 * @param Reference to the RRG
	 * @param If all current paths and distances should be reset or only a "local update" is necessary
	 * @param Reference to list of nodes indices which should be updated, newly reachable nodes will be
	 * added to this list
	 * @param Reference to if a node was added to the list of nodes to update
	 */
	void findBestRoutes(std::set<int> node_queue,
			rrg_nbv_exploration_msgs::Graph &rrg, bool reset,
			std::list<int> &nodes_to_update, bool &added_node_to_update);

	/**
	 * @brief Initialize a starting node for an update of all nodes' paths and costs, uses the yaw from
	 * the robot to this node as heading in
	 * @param Index of the start node
	 * @param Reference to the robot's pose
	 * @param Robot orientation in deg
	 * @param Reference to the RRG
	 */
	void initializeStartingNode(int start_node,
			const geometry_msgs::Pose &robot_pos, int robot_yaw,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Check if a given neighbor node is already in the current node's path
	 * @param Index of the neighbor node
	 * @param Index of the current node
	 * @param Reference to the RRG
	 * @param If the neighbor node is in the current node's path
	 */
	bool isNodeInPath(int neighbor_node_index, int current_node,
			rrg_nbv_exploration_msgs::Graph &rrg);
}
;
}
