#include "ros/ros.h"
#include <rrt_nbv_exploration_msgs/Tree.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include <octomap/octomap.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>

struct point {
	unsigned int x;
	unsigned int y;
};

namespace rrt_nbv_exploration {
/**
 * @brief The CollisionChecker class calculates a traversable path for the robot to a new node from it's parent node.
 */
class CollisionChecker {
public:
	/**
	 * @brief Constructor that initializes the node handle, parameters and a publisher for raytracing visualization
	 */
	CollisionChecker();
	/**
	 * @brief Checks if a feasible path from the nearest neighbour to the randomly sampled point exists for the particular robot by checking for collisions
	 * with fcl in the octomap
	 * @param Reference to a possible new node in the RRT
	 * @param Reference to the node that would be the nearest neighbour for a node with the given random position
	 * @param Randomly sampled position serving as a base for a new node's position
	 * @param Distance between the nearest node in the RRT and the randomly sampled position
	 * @return Returns true if a path (or a shorter path because of obstacles) between the nodes was found and false otherwise
	 */
	bool steer(rrt_nbv_exploration_msgs::Node &new_node,
			rrt_nbv_exploration_msgs::Node &nearest_node,
			geometry_msgs::Point rand_sample, double min_distance);

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
			rrt_nbv_exploration_msgs::Tree rrt, int start_node, int goal_node);

private:
	ros::NodeHandle _nh;
	ros::Publisher _collision_visualization;
	ros::Publisher _visualization_pub;
	ros::Subscriber _octomap_sub;
	ros::Subscriber _occupancy_grid_sub;
	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	std::shared_ptr<octomap::AbstractOcTree> _abstract_octree;
	std::shared_ptr<octomap::OcTree> _octree;

	nav_msgs::OccupancyGrid _occupancy_grid;

	/**
	 * @brief Required minimum distance between two nodes
	 */
	double _min_extend_range;
	/**
	 * Radius that includes robot's footprint in m
	 */
	double _robot_radius;
	/**
	 * Width of the robot in m
	 */
	double _robot_width;
	/**
	 * Height of the robot in m
	 */
	double _robot_height;
	/**
	 * Minimum required distance between two nodes for a box collision object to be inserted
	 */
	double _path_box_distance_thres;
	/**
	 * @brief Distance on z-axis between base footprint and sensor frame
	 */
	double _sensor_height;
	/**
	 * @brief Show collision checking
	 */
	bool _visualize_collision;
	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;
	/**
	 * If the occupancy map for visualization was initialized already
	 */
	bool _init_vis_map;
	/**
	 * Occupancy map for visualizing collision checking on a 2D grid
	 */
	nav_msgs::OccupancyGrid vis_map;

	/**
	 * @brief Function called by subscriber to "octomap_binary" message and converts it to the octree data format for further processing
	 * @param "octomap_binary" message
	 */
	void convertOctomapMsgToOctree(
			const octomap_msgs::Octomap::ConstPtr& map_msg);
	/**
	 * @brief Function called by subscriber to map message which saves the current occupancy grid for collision checking
	 * @param Map message
	 */
	void occupancyGridCallback(
			const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

	/**
	 * @brief Convert world coordinates to coordinates in the map
	 * @param World X-coordinate
	 * @param World Y-coordinate
	 * @param Reference to map X-coordinate that will be set
	 * @param Reference to map Y-coordinate that will be set
	 * @param Reference to map for conversion
	 */
	bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my, nav_msgs::OccupancyGrid &map);

	/**
	 * @brief Check if a circular area with the given center is in collision
	 * @param X-coordinate of the circle's center
	 * @param Y-coordinate of the circle's center
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @return True if a collision was registered, false otherwise
	 */
	bool isCircleInCollision(double x, double y, nav_msgs::OccupancyGrid &map, nav_msgs::OccupancyGrid &vis_map);

	/**
	 * @brief Check if a rotated rectangular area with the given center and yaw rotation is in collision
	 * @param X-coordinate of the rectangle's center
	 * @param Y-coordinate of the rectangle's center
	 * @param Yaw rotation of the rectangle around its center
	 * @param Height of the rectangle divided by 2
	 * @param Width of the rectangle divided by 2
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @return True if a collision was registered, false otherwise
	 */
	bool isRectangleInCollision(double x, double y, double yaw, double half_height, double half_width, nav_msgs::OccupancyGrid &map, nav_msgs::OccupancyGrid &vis_map);

	/**
	 * @brief Check if a line from one y-coordinate to another with consistent x-coordinate is in collision
	 * @param Starting y-coordinate
	 * @param Ending y-coordinate
	 * @param X-coordinate
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @return True if a collision was registered, false otherwise
	 */
	bool isLineInCollision(int y_start, int y_end, int x, nav_msgs::OccupancyGrid &map, nav_msgs::OccupancyGrid &vis_map);

	/**
	 * @brief Visualize collision objects as markers in RViz
	 * @param Start node position
	 * @param Goal node position
	 * @param Center position in the middle of start and goal
	 * @param Distance between start and goal in m
	 * @param Yaw between positions in rad
	 * @param Collision occured at start cylinder
	 * @param Collision occured at goal cylinder
	 * @param Collision occured in path box
	 */
	void visualizeCollisionCheck(geometry_msgs::Point start,
			geometry_msgs::Point goal, geometry_msgs::Point center,
			double distance, double yaw, bool collision_start,
			bool collision_goal, bool collision_path);
};
}
