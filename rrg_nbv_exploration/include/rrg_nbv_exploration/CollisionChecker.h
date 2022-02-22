#include "ros/ros.h"
#include <rrg_nbv_exploration_msgs/Node.h>
#include <rrg_nbv_exploration_msgs/Edge.h>
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rrg_nbv_exploration/GraphSearcher.h>
#include <rrg_nbv_exploration/GraphPathCalculator.h>

#define PI_HALF (M_PI / 2)

/**
 * @brief Directions for evasive inflation
 */
enum Directions {
	none = -1,
	center = 1000,
	north = 0,
	south = 180,
	east = 270,
	west = 90,
	northeast = 315,
	northwest = 45,
	southeast = 225,
	southwest = 135
};

/**
 * @brief Different collision detections
 */
enum Collisions {
	empty = 0, unknown = 1, occupied = 2
};

/**
 * @brief Structure to store offset that define a slice parallel to the x-axis of a circle or ring
 * in the grid map
 */
struct CircleLine {
	unsigned int x_offset;
	unsigned int x_start;
	unsigned int y_offset;

	CircleLine(unsigned int x, unsigned int y) {
		x_offset = x;
		y_offset = y;
		x_start = 0;
	}

	CircleLine(unsigned int xs, unsigned int x, unsigned int y) {
		x_offset = x;
		y_offset = y;
		x_start = xs;
	}
};

struct GridPoint {
	unsigned int x;
	unsigned int y;
};

struct MapPoint {
	double x;
	double y;
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
	 * @brief Initialize collision checking visualization if active and tries to inflate root node if active
	 * @param Reference to RRG
	 * @param Helper class for nearest neighbor search in the RRG
	 * @param Helper class for path calculation in the RRG
	 */
	void initialize(rrg_nbv_exploration_msgs::Graph &rrg,
			std::shared_ptr<GraphSearcher> graph_searcher,
			std::shared_ptr<GraphPathCalculator> graph_path_calculator);

	/**
	 * @brief Checks if a feasible path from the nearest neighbors to the randomly sampled point exists
	 * for the particular robot by checking for collisions in the subscribed occupancy grid map
	 * @param Reference to the RRG
	 * @param Reference to a possible new node in the RRT
	 * @param Randomly sampled position serving as a base for a new node's position
	 * @param Reference to robot pose
	 * @return Returns true if a path between the nodes was found and false otherwise
	 */
	bool steer(rrg_nbv_exploration_msgs::Graph &rrg,
			rrg_nbv_exploration_msgs::Node &new_node,
			geometry_msgs::Point rand_sample, geometry_msgs::Pose &robot_pos);

	/**
	 * @brief Try to inflate an existing node which inflation was previously stopped because of
	 * unknown tiles
	 * @param Reference to the RRG
	 * @param Index of the node to be inflated
	 * @param Pose of the robot
	 */
	void inflateExistingNode(rrg_nbv_exploration_msgs::Graph &rrg, int node,
			geometry_msgs::Pose robot_pos);

	/**
	 * @brief Check if a previously failed node is in collision
	 * @param Reference to the RRG
	 * @param Index of the node to be checked
	 * @return Type of detected collision (empty, unknown or occupied)
	 */
	int collisionCheckForFailedNode(rrg_nbv_exploration_msgs::Graph &rrg,
			int node);

	/**
	 * @brief Retry edges that previously failed the collision check due to unknown tiles
	 * @param Reference to the RRG
	 * @param Pose of the robot
	 */
	void retryEdges(rrg_nbv_exploration_msgs::Graph &rrg,
			geometry_msgs::Pose robot_pos);

private:
	ros::NodeHandle _nh;
	ros::Publisher _collision_visualization;
	ros::Publisher _visualization_pub;
	ros::Subscriber _occupancy_grid_sub;
	ros::Subscriber _occupancy_grid_updates_sub;

	/**
	 * @brief Helper class for kd-tree GraphConstructor and nearest neighbor search
	 */
	std::shared_ptr<GraphSearcher> _graph_searcher;
	/**
	 * @brief Helper class for calculating path, heading and traversability between any two nodes in
	 * the graph
	 */
	std::shared_ptr<GraphPathCalculator> _graph_path_calculator;

	std::shared_ptr<nav_msgs::OccupancyGrid> _occupancy_grid;

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
	 * @brief Maximal sensor range that is considered for gain calculation
	 */
	double _sensor_range;
	/**
	 * If the occupancy map for visualization was initialized already
	 */
	bool _init_vis_map;
	/**
	 * Occupancy map for visualizing collision checking on a 2D grid
	 */
	nav_msgs::OccupancyGrid _vis_map;
	/**
	 * If the initial position when starting exploration has to be checked for obstacles
	 */
	bool _check_init_position;
	/**
	 * @brief Grid map cell edge length in m
	 */
	double _grid_map_resolution;
	/**
	 * @brief Grid map value that indicates a cell is occupied (or inscribed)
	 */
	int _grid_map_occupied;
	/**
	 * @brief Grid map value that indicates a cell is unknown
	 */
	int _grid_map_unknown;
	/**
	 * @brief If the inflation of nodes (wavefront) is active
	 */
	bool _inflation_active;
	/**
	 * @brief If nodes can be moved away from obstacles during inflation
	 */
	bool _move_nodes;
	/**
	 * @brief Squared min distance between two nodes in the graph
	 */
	double _min_edge_distance_squared;
	/**
	 * @brief Max distance between two nodes in the graph
	 */
	double _max_edge_distance;
	/**
	 * @brief Squared max distance between two nodes in the graph
	 */
	double _max_edge_distance_squared;
	/**
	 * @brief X and y offsets from the circle's center that form the edge of the circle with the
	 * robot's radius, only contains positive y-offsets, negative ones are symmetrical
	 */
	std::vector<CircleLine> _circle_lines_offset;
	/**
	 * @brief List of pairs of radius and and respective x and y offsets from the circle's center that
	 * form a ring around the edge of the circle with the next smaller radius, only contains positive
	 * y-offsets, negative ones are symmetrical
	 */
	std::vector<std::pair<double, std::vector<CircleLine>>> _inflated_ring_lines_offsets;
	/**
	 * @brief List of radius and respective path box distance threshold that matches the particular radius
	 */
	std::vector<std::pair<double, double>> _path_box_distance_thresholds;
	/**
	 * @brief List of edges that could not be placed due to unknown map tiles in their path box and
	 * which could be retried
	 */
	std::vector<rrg_nbv_exploration_msgs::Edge> _retriable_edges;

	ros::Publisher _rrt_collision_visualization_pub;
	visualization_msgs::MarkerArray _node_points;
	visualization_msgs::MarkerArray _node_edges;
	int _marker_id;

	/**
	 * @brief Function called by subscriber to map message which saves the current occupancy grid for collision checking
	 * @param Map message
	 */
	void occupancyGridCallback(
			const nav_msgs::OccupancyGrid::ConstPtr &map_msg);

	/**
	 * @brief Helper function to retrieve the occupancy grid's index for updating it
	 * @param x coordinate
	 * @param y coordinate
	 * @return Index of the occupancy grid data
	 */
	int getIndex(int x, int y);

	/**
	 * @brief Function called by subscriber to map message which saves the current occupancy grid update for collision checking
	 * @param Map message
	 */
	void occupancyGridUpdatesCallback(
			const map_msgs::OccupancyGridUpdate::ConstPtr &map_msg);

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
	 * @brief Calculate the x and y offsets from a circle's center in the grid
	 * @param Radius of the circle to calculate
	 * @return Set of x and y offsets that describe the edge of the circle in the grid
	 */
	std::vector<CircleLine> calculateCircleLinesOffset(double radius);

	/**
	 * @brief Calculate the x and y offsets of the edge of the circle in the grid for the robot radius
	 */
	void precalculateCircleLinesOffset();

	/**
	 * @brief Calculate the x and y offsets and the x start for the current inflation radius up to the
	 * sensor range
	 */
	void calculateNextInflatedCircleLinesOffset();

	/**
	 * @brief Check if a set of offsets with the given center is in collision
	 * @param X-coordinate of the set's center
	 * @param Y-coordinate of the set's center
	 * @param Reference to if the direction can be merged
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @param Set of offsets to check
	 * @param Reference to the traversability cost of this line
	 * @param Reference to the number of tiles in this line
	 * @param If this is true, recheck and calculate traversability and tiles for whole inflated circle
	 * @param Reference to the detected collision (0=free, 1=unknown, 2=occupied)
	 * @return Return the direction if a collision was detected, 0 otherwise
	 */
	int isSetInCollision(double center_x, double center_y,
			bool &fixed_direction, nav_msgs::OccupancyGrid &map,
			std::vector<int8_t> &vis_map, std::vector<CircleLine> offsets,
			int &cost, int &tiles, int &collision, bool recalculate = false);

	/**
	 * @brief Check if a circular area with the given center is in collision
	 * @param X-coordinate of the circle's center
	 * @param Y-coordinate of the circle's center
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @param Reference to the traversability cost of this line
	 * @param Reference to the number of tiles in this line
	 * @param Reference to the detected collision (0=free, 1=unknown, 2=occupied)
	 * @return True if a collision was registered, false otherwise
	 */
	bool isCircleInCollision(double center_x, double center_y,
			nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map,
			int &cost, int &tiles, int &collision);

	/**
	 * @brief Check if a rotated rectangular area with the given center and yaw rotation is in collision
	 * @param X-coordinate of the rectangle's center
	 * @param Y-coordinate of the rectangle's center
	 * @param Yaw rotation of the rectangle around its center
	 * @param Height of the rectangle divided by 2
	 * @param Width of the rectangle divided by 2
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @param Reference to the traversability cost of this line
	 * @param Reference to the number of tiles in this line
	 * @param Reference to the detected collision (0=free, 1=unknown, 2=occupied)
	 * @return True if a collision was registered, false otherwise
	 */
	bool isRectangleInCollision(double x, double y, double yaw,
			double half_height, double half_width, nav_msgs::OccupancyGrid &map,
			std::vector<int8_t> &vis_map, int &cost, int &tiles,
			int &collision);

	/**
	 * @brief Check if an aligned rectangular area with the given center and yaw rotation is in collision
	 * @param X-coordinate of the rectangle's center
	 * @param Y-coordinate of the rectangle's center
	 * @param Yaw rotation of the rectangle around its center
	 * @param Height of the rectangle divided by 2
	 * @param Width of the rectangle divided by 2
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @param Reference to the traversability cost of this line
	 * @param Reference to the number of tiles in this line
	 * @param Reference to the detected collision (0=free, 1=unknown, 2=occupied)
	 * @return True if a collision was registered, false otherwise
	 */
	bool isAlignedRectangleInCollision(double x, double y, double yaw,
			double half_height, double half_width, nav_msgs::OccupancyGrid &map,
			std::vector<int8_t> &vis_map, int &cost, int &tiles,
			int &collision);

	/**
	 * @brief Check how far a circle with the given center can be inflated until a collision is detected
	 * @param Reference to x-coordinate of the circle's center
	 * @param Reference to y-coordinate of the circle's center
	 * @param If the node should be moved during inflation
	 * @param Reference nearest node to the new node
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @param Reference to the traversability cost of this line
	 * @param Reference to the number of tiles in this line
	 * @param Reference to the detected collision (0=free, 1=unknown, 2=occupied)
	 * @param Radius at which to start the inflation
	 * @param Max radius at which inflation must stop
	 * @return Maximum radius of the inflated circle
	 */
	double inflateCircle(double &x, double &y, bool move_node,
			rrg_nbv_exploration_msgs::Node &nearest_node,
			nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map,
			int &cost, int &tiles, int &collision, double current_radius,
			double max_radius);

	/**
	 * @brief Compares the current direction with the new direction from a collision and
	 * returns the merged direction or none if they are contrary
	 * @param Reference to current direction the circle will be moved to which will be overwritten
	 * with the merged direction (none for contrary directions)
	 * @param If the direction is fixed and cannot be merged anymore
	 * @param New direction the circle need to move to because of a collision (cannot be center or none)
	 * @return False if a contradictory direction was found, true otherwise
	 */
	bool checkDirection(int &current_direction, bool &fixed, int new_direction);

	/**
	 * @brief Round the given points coordinates to be in the middle of a grid cell (necessary for collision checking)
	 * @param Reference to a point which position is aligned
	 */
	void alignPointToGridMap(geometry_msgs::Point &rand_sample);

	/**
	 * @brief Calculate the size of the path box edge and returns if it must be checked for collision
	 * @param Index of the nearest node to the new node
	 * @param Reference to the path box's length which will be calculated here
	 * @param Distance between the nearest node and the new node in m
	 * @param Reference to the path box edge's center position which will be calculated here
	 * @param Reference to the new node
	 * @param Reference to the RRG
	 * @return If the path box edge must be checked or if it is entirely inside both node areas
	 */
	bool calculateEdge(int nearest_node, double &edge_length, double distance,
			geometry_msgs::Point &edge_center,
			rrg_nbv_exploration_msgs::Node &new_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Check if a point at the given position would be engulfed by other nodes' radii
	 * @param Reference to a point for which the check will be done
	 * @param Reference to the nearest node to the point regarding the radius which will be set
	 * @param Reference to the RRG
	 * @return If the point is not engulfed by other nodes' radii
	 */
	bool checkEngulfing(geometry_msgs::Point &point, int &nearest_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Check if a point at the given position meets the required minimum distance between nodes
	 * and if it is not further away than the maximum distance (replaces the point at max distance otherwise)
	 * @param Reference to a point for which the check will be done
	 * @param Reference to the nearest node to the point which will be set
	 * @param Reference to the RRG
	 * @return If the point is further away than the minimum required distance between nodes
	 */
	bool checkDistance(geometry_msgs::Point &point, int &nearest_node,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Try to move a circle one grid map tile in the given direction
	 * @param Reference to x-coordinate of the circle's center
	 * @param Reference to y-coordinate of the circle's center
	 * @param Direction in which the circle will be moved
	 * @param Index of the ring line offset that is currently being evaluated
	 * @param List of positions the circle was at before (abort if new position is a previous one)
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @param Reference to the traversability cost of this line
	 * @param Reference to the number of tiles in this line
	 * @param Reference to the detected collision (0=free, 1=unknown, 2=occupied)
	 * @return True if moving circle was successful, false otherwise
	 */
	bool moveCircle(double &x, double &y, int direction, int ring,
			std::vector<std::pair<double, double>> &previous_positions,
			nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map,
			int &cost, int &tiles, int &collision);

	/**
	 * @brief Validate that movement in the given direction has the new node remain in nearest node's
	 * connectable zone (between engulfing and inside the path box threshold)
	 * @param Reference to x-coordinate of the new node's center
	 * @param Reference to y-coordinate of the new node's center
	 * @param Direction in which the new node will be moved
	 * @param Reference to the nearest node
	 * @param Reference to if the direction can be merged
	 * @return If the movement in the given direction is acceptable or moves the new node out of connection
	 */
	bool checkMovementWithNearestNode(double &x, double &y, int &direction,
			rrg_nbv_exploration_msgs::Node &nearest_node,
			bool &fixed_direction);

	/**
	 * @brief Check if a line from one x-coordinate to another with consistent y-coordinate is in collision
	 * @param Starting x-coordinate
	 * @param Ending x-coordinate
	 * @param y-coordinate
	 * @param Reference to the map for checking collision
	 * @param Reference to the visualization map to display checked areas
	 * @param Reference to the traversability cost of this line
	 * @param Reference to the number of tiles in this line
	 * @param Reference to the detected collision (0=free, 1=unknown, 2=occupied)
	 * @return If a collision was detected
	 */
	bool isLineInCollision(int x_start, int x_end, int y,
			nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map,
			int &cost, int &tiles, int &collision);
	/**
	 * @brief Initialize collision visualization map
	 * @param Occupancy grid map that is checked during steering
	 */
	void initVisMap(const nav_msgs::OccupancyGrid &map);

	/**
	 * @brief Initialize the root node and all values to calculate the reward function
	 * @param Occupancy grid map that is checked during steering
	 * @param Reference to the RRG
	 */
	void initRootNodeAndGraph(nav_msgs::OccupancyGrid &map,
			rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Add a visualization marker for the given edge to the list of edge markers
	 * @param Length of the edge marker
	 * @param Center coordinates of the edge
	 * @param Yaw of the edge
	 * @param Reference to the list of edge markers
	 */
	void visualizeEdge(double edge_length,
			const geometry_msgs::Point &edge_center, double edge_yaw,
			std::vector<visualization_msgs::Marker> &new_edge_markers);

	/**
	 * @brief Add a visualization marker for the given node
	 * @param Node object to be visualized
	 */
	void visualizeNode(rrg_nbv_exploration_msgs::Node &node, int failed = 0);

	/**
	 * @brief Returns a point with coordinates of moving given x and y in the provided direction
	 * @param Direction in which the point should move to the next tile center in degrees
	 * @param Starting x-coordinate
	 * @param Starting y-coordinate
	 * @return Moved point
	 */
	geometry_msgs::Point movePoint(int direction, double &x, double &y);

	/**
	 * @brief Constructs a new edge from the given node to the new node being place
	 * @param Yaw of the edge
	 * @param Length of edge's path box
	 * @param Traversability cost of the edge's path box
	 * @param Number of tiles inside the edge's path box
	 * @param First node of the edge
	 * @param Distance between the two nodes
	 * @param Reference to the RRG
	 * @return The constructed edge
	 */
	rrg_nbv_exploration_msgs::Edge constructEdge(double edge_yaw,
			double edge_length, int edge_cost, int edge_tiles, int node,
			double distance, rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Check if the newly added edge shortens the path length to the robot for one of the nodes
	 * of this edge
	 * @param Robot pose
	 * @param Reference to the RRG
	 * @param the newly added edge
	 */
	void checkNewEdgePathLength(const geometry_msgs::Pose &robot_pos,
			rrg_nbv_exploration_msgs::Graph &rrg,
			rrg_nbv_exploration_msgs::Edge &edge);
};
}
