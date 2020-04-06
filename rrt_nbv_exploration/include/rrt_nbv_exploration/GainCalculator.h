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
	 * Calculates the gain of the passed node by raytracing in the octree
	 * @param Node which gain needs to be calculated
	 * @param Pointer to octree for raytracing
	 */
	void calculateGain(rrt_nbv_exploration_msgs::Node &node,
			std::shared_ptr<octomap::OcTree> octree);

	void recalculateGain(rrt_nbv_exploration_msgs::Tree &rrt,
			std::vector<int> nodes, std::shared_ptr<octomap::OcTree> octree);
private:
	ros::NodeHandle _nh;
	ros::Publisher _raycast_visualization;
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
	 * Pre-calculates lists of all gain poll points in cartesian coordinates based on theta and phi steps as well as radial steps
	 */
	void precalculateGainPollPoints();
};
}
