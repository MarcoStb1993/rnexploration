#include "ros/ros.h"
#include <rrg_nbv_exploration_msgs/Node.h>
#include <rrg_nbv_exploration_msgs/Cluster.h>
#include <rrg_nbv_exploration_msgs/NodeToUpdate.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include "visualization_msgs/Marker.h"

#include <boost/multi_array.hpp>
#include <fstream>
#include <queue>

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

typedef boost::multi_array<int, 3> cluster_point_array;
typedef cluster_point_array::index cluster_point_array_index;

/**
 * Label for cluster points
 */
enum ClusterLabel {
	NoPoint = -2, Noise = -1, Undefined = 0
};

/**
 * Indices for a point in the multi array of cluster points
 */
struct ClusterIndex {
	cluster_point_array_index theta, phi, radius;
};

namespace rrg_nbv_exploration {

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
	void calculateGain(rrg_nbv_exploration_msgs::Node &node);

private:
	ros::NodeHandle _nh;
	ros::Publisher raysample_visualization;
	ros::Publisher cluster_visualization;
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
	rrg_nbv_exploration_msgs::Node _last_updated_node;
	/**
	 * @brief Resolution of octomap (edge length of voxels in m)
	 */
	double _octomap_resolution;
	/**
	 * @brief Max plausible/acceptable height difference between the node's initial height and the measured height by raytracing
	 */
	double _max_node_height_difference;
	/**
	 * @brief Number of nodes required as neighbors to classify as core point in DBSCAN
	 */
	int _dbscan_min_points;
	/**
	 * @brief Distance between points to count as neighbors for DBSCAN
	 */
	double _dbscan_epsilon;

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
			const rrg_nbv_exploration_msgs::NodeToUpdate::ConstPtr &node_to_update);

	/**
	 * @brief Calculates the gain of the passed node by sparse ray polling in the octree
	 * @param Node which gain needs to be calculated
	 */
	void calculatePointGain(rrg_nbv_exploration_msgs::Node &node);

	void initializeCluster(int cluster_counter,
			rrg_nbv_exploration_msgs::Cluster &current_cluster);

	void addPointToCluster(rrg_nbv_exploration_msgs::Cluster &current_cluster,
			double &center_theta_sum_sin, double &center_theta_sum_cos,
			double theta, double phi, double radius);

	void finishCluster(rrg_nbv_exploration_msgs::Cluster &current_cluster,
			double center_theta_sum_sin, double center_theta_sum_cos);

	/**
	 * @brief Add the visualization of a cluster point to the visualization array
	 * @param Theta index for point to visualize
	 * @param Phi index for point to visualize
	 * @param Radius index for point to visualize
	 * @param X position of node
	 * @param Y position of node
	 * @param Z position of node
	 * @param Number of cluster to which the point belongs
	 * @param Array of visualization points to append cluster point visualization to
	 */
	void addClusterVisualizationPoint(cluster_point_array_index theta,
			cluster_point_array_index phi, cluster_point_array_index radius,
			double x, double y, double z, int cluster_counter,
			visualization_msgs::Marker &cluster_points_vis);

	/**
	 * @brief Request the 6 direct neighbors of the given point and adds all existing keys for them
	 * @param Index for point which neighbors must be retrieved
	 * @param Multi array with all cluster points
	 * @param Queue of all indices of neighbors to check for cluster
	 */
	void retrieveClusterPointNeighbors(ClusterIndex point,
			cluster_point_array &cluster_points,
			std::queue<ClusterIndex> &neighbor_keys);

	/**
	 * @brief Checks if there is a cluster point at the given index
	 * @param Cluster point index to check
	 * @param Multi array with all cluster points
	 * @param List of neighbors to add it to if it exists
	 */
	void checkIfClusterPointExists(ClusterIndex index,
			cluster_point_array &cluster_points,
			std::vector<ClusterIndex> &neighbors);

	/**
	 * @brief Checks whether two cluster are close to each other by comparing theta, phi and radius
	 * position with a margin as big as the step sizes
	 * @param Cluster one
	 * @param Cluster two
	 * @return Returns true if clusters are close to each other
	 */
	bool clusterProximityCheck(rrg_nbv_exploration_msgs::Cluster &cluster1,
			rrg_nbv_exploration_msgs::Cluster &cluster2);

	/**
	 * Measures the likely z coordinate of the node by raytracing in the octree (first measures downward from the
	 * node's initial position until it finds the ground, then sets the z coordinate at sensor height above ground -
	 * if no collision occurs, raytraces upward and does the same)
	 * @param Node which height needs to be measured
	 * @return If height could be measured
	 */
	bool measureNodeHeight(rrg_nbv_exploration_msgs::Node &node);
};
}
