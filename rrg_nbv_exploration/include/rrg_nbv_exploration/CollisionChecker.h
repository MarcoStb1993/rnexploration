#include "ros/ros.h"
#include <rrg_nbv_exploration_msgs/Node.h>
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI_HALF (M_PI / 2)

struct GridPoint {
	unsigned int x;
	unsigned int y;
};

struct MapPoint {
	double x;
	double y;
};

struct CircleLine {
	unsigned int x_offset;
	unsigned int y_offset;

	CircleLine(unsigned int x, unsigned int y) {
		x_offset = x;
		y_offset = y;
	}
};

namespace rrg_nbv_exploration {
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
	 * @param If the circle around the sample point should be checked
	 * @return Returns true if a path (or a shorter path because of obstacles) between the nodes was found and false otherwise
	 */
	bool steer(rrg_nbv_exploration_msgs::Node &new_node,
			rrg_nbv_exploration_msgs::Node &nearest_node,
			geometry_msgs::Point rand_sample, double min_distance,
			double check_circle);

	/**
	 * @brief Initialize collision checking visualization if active and checks circle around robot if activated
	 * @param Robot's position where RRT root is placed
	 * @return If initialization succeeded
	 */
	bool initialize(geometry_msgs::Point position);

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
	 * If the occupancy map for visualization was initialized already
	 */
	bool _init_vis_map;
	/**
	 * Occupancy map for visualizing collision checking on a 2D grid
	 */
	nav_msgs::OccupancyGrid vis_map;
	/**
	 * If the initial position when starting exploration has to be checked for obstacles
	 */
	bool _check_init_position;
	/**
	 * @brief Grid map cell edge length in m
	 */
	double _grid_map_resolution;

	std::vector<CircleLine> _circle_lines_offset;

	ros::Publisher _rrt_collision_visualization_pub;
	visualization_msgs::MarkerArray _node_points;
	visualization_msgs::MarkerArray _node_edges;

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

	void precalculateCircleLinesOffset(std::vector<int8_t> &vis_map);

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
	 * @brief Check if an aligned rectangular area with the given center and yaw rotation is in collision
	 * @param X-coordinate of the rectangle's center
	 * @param Y-coordinate of the rectangle's center
	 * @param Yaw rotation of the rectangle around its center
	 * @param Height of the rectangle divided by 2
	 * @param Width of the rectangle divided by 2
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @return True if a collision was registered, false otherwise
	 */
	bool isAlignedRectangleInCollision(double x, double y, double yaw,
			double half_height, double half_width, nav_msgs::OccupancyGrid &map,
			std::vector<int8_t> &vis_map);

	/**
	 * @brief Check if a line from one x-coordinate to another with consistent y-coordinate is in collision
	 * @param Starting x-coordinate
	 * @param Ending x-coordinate
	 * @param y-coordinate
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @return True if a collision was registered, false otherwise
	 */
	bool isLineInCollision(int x_start, int x_end, int y,
			nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map);
	/**
	 * @brief Initialize collision visualization map
	 * @param Occupancy grid map that is checked during steering
	 */
	void initVisMap(const nav_msgs::OccupancyGrid &map);
};
}
