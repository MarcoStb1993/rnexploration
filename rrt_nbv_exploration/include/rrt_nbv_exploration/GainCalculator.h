#include "ros/ros.h"
#include "rrt_nbv_exploration_msgs/Tree.h"
#include "rrt_nbv_exploration_msgs/Node.h"
#include <rrt_nbv_exploration_msgs/NodeToUpdate.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include "visualization_msgs/Marker.h"
#include <rrt_nbv_exploration/GainCalculatorConfig.h>

#include <boost/multi_array.hpp>
#include <fstream>

/**
 * Structure to store the step and corresponding cosine and sine value for gain calculation speedup
 */
struct StepStruct {
	int step;
	double cos;
	double sin;
};

/**
 * Point in Cartesian coordinates that includes theta (azimuth) and phi (polar) angles in degrees,
 * radius in meters from the spherical calculation and the information if the poll point is inside
 * the sensor's sensing range (adds to gain) or just to check if there are no obstacles directly in
 * front of the sensor
 */
struct PollPoint {
	double x, y, z;
	int theta, phi;
	double radius;
	bool in_range;
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
	 * Constructor that initializes the node handle, parameters and a publisher for sparse ray polling visualization
	 */
	GainCalculator();
	~GainCalculator();

	/**
	 * Pre-calculates lists of all gain poll points/rays in cartesian coordinates based on theta and phi steps as well as radial steps depending on the mode
	 */
	void precalculateGainPolls();

	/**
	 * Calculates the gain of the passed node the selected gain calculation method
	 * @param Node which gain needs to be calculated
	 */
	void calculateGain(rrt_nbv_exploration_msgs::Node &node);

	void dynamicReconfigureCallback(
			rrt_nbv_exploration::GainCalculatorConfig &config, uint32_t level);

private:
	ros::NodeHandle _nh;
	ros::Publisher raysample_visualization;
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
	 * Sensor's vertical FoV bottom edge that is considered for gain calculation (in degrees, from 180 to 0 as the highest angle)
	 */
	int _sensor_vertical_fov_bottom;
	/**
	 * Sensor's vertical FoV top edge that is considered for gain calculation (in degrees, from 180 to 0 as the highest angle)
	 */
	int _sensor_vertical_fov_top;
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
	 * A pre-calculated 3-dimensional array (theta, phi, radius) of all points to poll for gain calculation
	 */
	multi_array _gain_poll_points;
	/**
	 * @brief Distance on z-axis between base footprint and sensor frame
	 */
	double _sensor_height;
	/**
	 * @brief Radius in m of a sphere that best describes the sensor's volume (checking for obstacles begins outside of this radius)
	 */
	double _sensor_size;
	/**
	 * @brief Sensor minimum range squared for faster distance comparison during raycasting
	 */
	double _sensor_min_range_squared;
	/**
	 * @brief Node which gain was calculated previously
	 */
	rrt_nbv_exploration_msgs::Node _last_updated_node;
	/**
	 * @brief Resolution of octomap (edge length of voxels in m)
	 */
	double _octomap_resolution;
	/**
	 * @brief Max plausible/acceptable height difference between the node's initial height and the measured height by raytracing
	 */
	double _max_node_height_difference;

	/**
	 * Pre-calculates lists of all gain poll points in cartesian coordinates based on theta and phi steps as well as radial steps
	 */
	void precalculateGainPollPoints();

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
			const rrt_nbv_exploration_msgs::NodeToUpdate::ConstPtr &node_to_update);

	/**
	 * Calculates the gain of the passed node by sparse ray polling in the octree
	 * @param Node which gain needs to be calculated
	 */
	void calculatePointGain(rrt_nbv_exploration_msgs::Node &node);

	/**
	 * Measures the likely z coordinate of the node by raytracing in the octree (first measures downward from the
	 * node's initial position until it finds the ground, then sets the z coordinate at sensor height above ground -
	 * if no collision occurs, raytraces upward and does the same)
	 * @param Node which height needs to be measured
	 * @return If height could be measured
	 */
	bool measureNodeHeight(rrt_nbv_exploration_msgs::Node &node);
};
}
