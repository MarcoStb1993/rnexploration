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
#include <tf2_ros/transform_listener.h>
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

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
	 * @brief Checks if a feasible path from the nearest neighbour to the randomly sampled point exists for the particular robot by raytracing in the octree
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

private:
	ros::NodeHandle _nh;
	ros::Publisher _steering_visualization;
	ros::Subscriber _octomap_sub;
	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	std::shared_ptr<octomap::AbstractOcTree> _abstract_octree;
	std::shared_ptr<octomap::OcTree> _octree;

	/**
	 * @brief Required minimum distance between two nodes
	 */
	double _min_extend_range;
	/**
	 * @brief Show raycasting for steering
	 */
	bool _visualize_steering;
	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;

	/**
	 * @brief Function called by subscriber to "octomap_binary" message and converts it to the octree data format for further processing
	 * @param "octomap_binary" message
	 */
	void convertOctomapMsgToOctree(
			const octomap_msgs::Octomap::ConstPtr& map_msg);
};
}
