#include "ros/ros.h"
#include <rrt_nbv_exploration_msgs/Tree.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

struct point {
	unsigned int x;
	unsigned int y;
};

namespace rrt_nbv_exploration {
/**
 * @brief The CollisionChecker class checks if a given position can be connected to the existing tree without collision on the occupancy grid.
 */
class CollisionChecker {
public:
	/**
	 * @brief Constructor that initializes the node handle, parameters and a publishers and subscribers
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

private:
	ros::NodeHandle _nh;
	ros::Publisher _collision_visualization;
	ros::Publisher _visualization_pub;
	ros::Subscriber _occupancy_grid_sub;

	nav_msgs::OccupancyGrid _occupancy_grid;

	/**
	 * Radius that includes robot's footprint in m
	 */
	double _robot_radius;
	/**
	 * Width of the robot in m
	 */
	double _robot_width;
	/**
	 * Minimum required distance between two nodes for a box collision object to be inserted
	 */
	double _path_box_distance_thres;
	/**
	 * @brief Show collision checking
	 */
	bool _visualize_collision;
	/**
	 * If the occupancy map for visualization was initialized already
	 */
	bool _init_vis_map;
	/**
	 * Occupancy map for visualizing collision checking on a 2D grid
	 */
	nav_msgs::OccupancyGrid vis_map;

	/**
	 * @brief Function called by subscriber to map message which saves the current occupancy grid for collision checking
	 * @param Map message
	 */
	void occupancyGridCallback(
			const nav_msgs::OccupancyGrid::ConstPtr &map_msg);

	/**
	 * @brief Convert world coordinates to coordinates in the map
	 * @param World X-coordinate
	 * @param World Y-coordinate
	 * @param Reference to map X-coordinate that will be set
	 * @param Reference to map Y-coordinate that will be set
	 * @param Reference to map for conversion
	 */
	bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my,
			nav_msgs::OccupancyGrid &map);

	/**
	 * @brief Check if a circular area with the given center is in collision
	 * @param X-coordinate of the circle's center
	 * @param Y-coordinate of the circle's center
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @return True if a collision was registered, false otherwise
	 */
	bool isCircleInCollision(double x, double y, nav_msgs::OccupancyGrid &map,
			std::vector<int8_t> &vis_map);

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
	bool isRectangleInCollision(double x, double y, double yaw,
			double half_height, double half_width, nav_msgs::OccupancyGrid &map,
			std::vector<int8_t> &vis_map);

	/**
	 * @brief Check if a line from one y-coordinate to another with consistent x-coordinate is in collision
	 * @param Starting y-coordinate
	 * @param Ending y-coordinate
	 * @param X-coordinate
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @return True if a collision was registered, false otherwise
	 */
	bool isLineInCollision(int y_start, int y_end, int x,
			nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map);

};
}
