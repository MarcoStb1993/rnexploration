#include "ros/ros.h"
#include "rrt_nbv_exploration_msgs/Tree.h"
#include "rrt_nbv_exploration_msgs/Node.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include "visualization_msgs/Marker.h"

#include <boost/multi_array.hpp>

/**
 * Structure to store the step and corresponding cosinus and sinus value for gain calculation speedup
 */
struct StepStruct {
	int step;
	double cos;
	double sin;
};

/**
 * Point in cartesian coordinates that includes theta (azimuth) and phi (polar) angles in degrees as
 * well as radius in meters from the spherical calculation
 */
struct PollPoint {
	double x, y, z;
	int theta, phi;
	double radius;
};

typedef boost::multi_array<PollPoint, 3> multi_array;
typedef multi_array::index multi_array_index;

namespace rrt_nbv_exploration {

/**
 * The gain calculator computes the gain of each node in the RRT.
 */
class GainCalculator {
public:
	/**
	 * Constructor that initializes the node handle, parameters and a publisher for raytracing visualization
	 */
	GainCalculator();
	~GainCalculator();

	/**
	 * Pre-calculates lists of all gain poll points in cartesian coordinates based on theta and phi steps as well as radial steps
	 */
	void precalculateGainPollPoints();

private:
	ros::NodeHandle _nh;
	ros::Publisher _raycast_visualization;
	ros::Publisher _updated_node_publisher;
	ros::Subscriber _node_to_update_subscriber;
	ros::Subscriber _octomap_sub;

	std::shared_ptr<octomap::AbstractOcTree> _abstract_octree;
	std::shared_ptr<octomap::OcTree> _octree;

	/**
	 * Maximal sensor range that is considered for gain calculation in m
	 */
	double _sensor_max_range;
	/**
	 * Minimal sensor range that is considered for gain calculation in m
	 */
	double _sensor_min_range;
	/**
	 * Delta element in phi direction (polar angle) in spherical coordinates in degrees
	 */
	int _delta_phi;
	/**
	 * Delta element in theta direction (azimuthal angle) in spherical coordinates in degrees
	 */
	int _delta_theta;
	/**
	 * Delta element in radial direction in spherical coordinates in m
	 */
	double _delta_radius;
	/**
	 * Sensor's horizontal FoV that is considered for gain calculation in degrees
	 */
	int _sensor_horizontal_fov;
	/**
	 * Sensor's vertical FoV that is considered for gain calculation in degrees
	 */
	int _sensor_vertical_fov;
	/**
	 * Maximum number of points that can be added for each yaw step to calculate gain
	 */
	int _best_gain_per_view;
	/**
	 * Maximum number of points that can be added to calculate gain
	 */
	int _max_gain_points;
	/**
	 * Required minimum percentage of points that have to be free space of all points in view used for gain calculation at a specific yaw angle
	 */
	double _min_view_score;
	/**
	 * Show gain calculation raycasting
	 */
	bool _visualize_gain_calculation;

	/**
	 * A pre-calculated 3-dimensional array (theta, phi, radius) of all points to poll for gain calculation
	 */
	multi_array _gain_poll_points;
	/**
	 * @brief Distance on z-axis between base footprint and sensor frame
	 */
	double _sensor_height;
	/**
	 * @brief Node which gain was calculated previously
	 */
	rrt_nbv_exploration_msgs::Node _last_updated_node;

	/**
	 * @brief Start gain calculation for first node in list of nodes to be updated
	 */
	void updateNodes();

	/**
	 * @brief Function called by subscriber to "octomap_binary" message and converts it to the octree data format for further processing
	 * @param "octomap_binary" message
	 */
	void convertOctomapMsgToOctree(
			const octomap_msgs::Octomap::ConstPtr &map_msg);

	/**
	 * @brief Callback for subscriber to "node_to_update" topic which delivers node to calculate the gain for
	 * @param Node which gain needs to be calculated
	 */
	void nodeToUpdateCallback(
			const rrt_nbv_exploration_msgs::Node::ConstPtr &node_to_update);

	/**
	 * Calculates the gain of the passed node by raytracing in the octree
	 * @param Node which gain needs to be calculated
	 */
	void calculateGain(rrt_nbv_exploration_msgs::Node &node);



};
}
