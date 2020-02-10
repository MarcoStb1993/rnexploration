#include "ros/ros.h"
#include "rrt_nbv_exploration_msgs/rrt.h"
#include "rrt_nbv_exploration_msgs/Node.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include "visualization_msgs/Marker.h"

namespace rrt_nbv_exploration
{
/**
 * @brief The gain calculator computes the gain of each node in the RRT.
 */
class GainCalculator {
public:
    /**
     * @brief Constructor that initializes the node handle, parameters and a publisher for raytracing visualization
     */
    GainCalculator();
    /**
     * @brief Calculates the gain of the passed node by raytracing in the octree
     * @param Node which gain needs to be calculated
     * @param Pointer to octree for raytracing
     */
    void calculate_gain(rrt_nbv_exploration_msgs::Node &node, boost::shared_ptr<octomap::OcTree> octree);

    void recalculate_gain(rrt_nbv_exploration_msgs::rrt &rrt, std::vector<int> nodes, boost::shared_ptr<octomap::OcTree> octree);
private:
    ros::NodeHandle _nh;
    ros::Publisher _raycast_visualization;
    /**
     * @brief Maximal sensor range that is considered for gain calculation
     */
    double _sensor_range;
    /**
     * @brief Show gain calculation raycasting
     */
    bool _visualize_gain_calculation;
};
}
