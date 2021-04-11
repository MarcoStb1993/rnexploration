#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include <rrt_nbv_exploration_msgs/Frontiers.h>
#include <rrt_nbv_exploration_msgs/Frontier.h>
#include <rrt_nbv_exploration_msgs/BestAndCurrentFrontier.h>
#include <rrt_nbv_exploration_msgs/RequestGoal.h>
#include <rrt_nbv_exploration_msgs/RequestPath.h>
#include <rrt_nbv_exploration_msgs/UpdateCurrentGoal.h>
#include <limits.h>
#include <random>
#include "math.h"
#include <rrt_nbv_exploration/FrontierComparator.h>
#include <rrt_nbv_exploration/GainCalculator.h>
#include <rrt_nbv_exploration/costmap_client.h>
#include <rrt_nbv_exploration/frontier_search.h>
#include <rrt_nbv_exploration/costmap_client.h>

namespace rrt_nbv_exploration {

/**
 * @brief The TreeConstructor class constructs an RRT at a given seed position and publishes it in the rrt_tree topic
 */
class TreeConstructor {
public:
	/**
	 * @brief Constructor that places the trees seed at (0,0,0)
	 */
	TreeConstructor();
	~TreeConstructor();
	/**
	 * @brief Initialization for ROS and tree TreeConstructor, called by the constructors
	 * @param Seed position for the tree
	 */
	void initialization(geometry_msgs::Point seed = geometry_msgs::Point());
	/**
	 * @brief Starts the tree TreeConstructor when stopped
	 */
	void startRrtConstruction();
	/**
	 * @brief Stops the tree TreeConstructor when running
	 */
	void stopRrtConstruction();
	/**
	 * @brief Tree TreeConstructor main function, publishes it in topic rrt_tree
	 * @param kd-tree for faster nearest neighbour queries
	 */
	void runRrtConstruction();

private:
	ros::NodeHandle _nh;

	ros::Publisher _frontiers_publisher;
	ros::Publisher _best_and_current_goal_publisher;
	ros::Subscriber _octomap_sub;
	ros::ServiceServer _request_goal_service;
	ros::ServiceServer _update_current_goal_service;
	ros::ServiceServer _set_rrt_state_service;
	ros::ServiceServer _get_rrt_state_service;
	ros::ServiceServer _reset_rrt_state_service;
	ros::Timer _exploration_finished_timer;

	std::shared_ptr<FrontierSearch> _search;
	std::shared_ptr<Costmap2DClient> _costmap_client;

	std::default_random_engine _generator;
	std::shared_ptr<octomap::AbstractOcTree> _abstract_octree;
	std::shared_ptr<octomap::OcTree> _octree;

	tf2_ros::Buffer _tf_buffer;
	tf2_ros::TransformListener _tf_listener;

	/**
	 * @brief Helper class for calculating the gain cost ratio of a frontier and sorting them ordered by this ratio
	 */
	std::shared_ptr<FrontierComparator> _frontier_comparator;
	/**
	 * @brief Helper class for calculating the gain of each frontier by checking occupancy in the OctoMap
	 */
	std::shared_ptr<GainCalculator> _gain_calculator;
	/**
	 * @brief Current frontiers and blocklisted frontiers
	 */
	rrt_nbv_exploration_msgs::Frontiers _frontiers;
	/**
	 * @brief State of the tree TreeConstructor (running or stopped)
	 */
	bool _running;
	/**
	 * @brief Current min value of bounding box of the 3D map as x, y and z value
	 */
	double _map_min_bounding[3] = { 0, 0, 0 };
	/**
	 * @brief Current max value of bounding box of the 3D map as x, y and z value
	 */
	double _map_max_bounding[3] = { 0, 0, 0 };
	/**
	 * @brief The frontier that is currently being pursued as a navigation goal
	 */
	int _current_goal_frontier;
	/**
	 * @brief Previously visited goal frontier
	 */
	int _last_goal_frontier;
	/**
	 * @brief Maximal sensor range that is considered for gain calculation
	 */
	double _sensor_range;
	/**
	 * @brief Doubled and squared sensor range for radius search in kd-tree
	 */
	double _radius_search_range;
	/**
	 * @brief Distance on z-axis between base footprint and sensor frame
	 */
	double _sensor_height;
	/**
	 * @brief Fixed length of the tree's edges, flexible if set to -1 (TODO: wavefront if set to 0)
	 */
	double _edge_length;
	/**
	 * @brief Previous recorded robot pose
	 */
	geometry_msgs::Pose _last_robot_pos;
	/**
	 * @brief Time until exploration counts as finished because no new frontiers were placed
	 */
	double _exploration_finished_timer_duration;
	/**
	 * @brief If the robot already moved past a neighbor frontier towards the current goal
	 */
	bool _moved_to_current_goal;
	/**
	 * @brief Indicator if current goal was already updated before a new goal can be requested
	 */
	bool _goal_updated;
	/**
	 * @brief Radius that includes robot's footprint in m
	 */
	double _robot_radius;
	/**
	 * @brief If current goal is currently updating
	 */
	bool _updating;
	/**
	 * @brief Squared distance in m the robot can move before frontiers are updated
	 */
	double _distance_thresh_squared;
	/**
	 * @brief Distance in degrees the robot can turn before frontiers are updated
	 */
	double _orientation_thresh;
	/**
	 * @brief Squared distance tolerance between old and new frontier to count them as the same in m
	 */
	double _frontier_tolerance_squared;
	/**
	 * @brief Minimum size for a frontier to be counted as such
	 */
	double _min_frontier_size;
	/**
	 * @brief If robot moved and frontiers must be updated
	 */
	bool _update_frontiers;
	/**
	 * @brief Name of the robot's tf frame
	 */
	std::string _robot_frame;

	/**
	 * @brief Initialize the frontier search at the robot's position, initialize helper classes and frontiers ordered by gain list with root frontier
	 * @param Seed position for frontier search
	 * @return If initialization was successful
	 */
	bool initRrt(const geometry_msgs::Point &seed);
	/**
	 * @brief Find frontiers in grid map
	 * @param Starting center for frontier search
	 */
	void findFrontiers(const geometry_msgs::Point &center);
	/**
	 * @brief Check if a given frontier is already on the blocklist
	 * @param New frontier
	 * @return If the frontier is on the blocklist
	 */
	bool isFrontierBlocklisted(rrt_nbv_exploration_msgs::Frontier new_frontier);

	/**
	 * @brief Check if a given frontier is the current goal
	 * @param New frontier
	 * @param Current goal frontier
	 * @return If the frontier is the current goal
	 */
	bool isFrontierCurrentGoal(rrt_nbv_exploration_msgs::Frontier new_frontier,
			rrt_nbv_exploration_msgs::Frontier current_goal);

	/**
	 * @brief Check if a given frontier is already in the list of frontiers
	 * @param New frontier
	 * @param Existing frontiers
	 * @return Index in existing frontiers if the frontier is already in the list of frontiers, -1 otherwise
	 */
	int isFrontierPresent(rrt_nbv_exploration_msgs::Frontier new_frontier,
			std::vector<rrt_nbv_exploration_msgs::Frontier> old_frontiers);

	/**
	 * @brief Check if a given frontier is out of two times the sensor range
	 * @param New frontier
	 * @return If the frontier is out of two times the sensor range
	 */
	bool isFrontierOutOfSensorRange(rrt_nbv_exploration_msgs::Frontier new_frontier);

	/**
	 * @brief Check if there is a current goal, if there are still frontiers to be explored and select a new goal if required and possible
	 */
	void checkCurrentGoal();
	/**
	 * @brief Update the last known robot position and check if the robot moved a minimum distance
	 */
	void determineIfRobotMoved();
	/**
	 * @brief Publish the frontier that currently has the best gain-cost-ratio
	 */
	void publishFrontierWithBestGain();
	/**
	 * @brief Returns the robot's pose in the map frame
	 * @return Robot pose
	 */
	geometry_msgs::Pose getRobotPose();
	/**
	 * @brief Function called by subscriber to "octomap_binary" message and converts it to the octree data format for further processing
	 * @param "octomap_binary" message
	 */
	void convertOctomapMsgToOctree(
			const octomap_msgs::Octomap::ConstPtr &map_msg);
	/**
	 * @brief Updates the map's dimension (x,y,z) after a new octomap was received
	 */
	void updateMapDimensions();
	/**
	 * @brief Updates the current goal and the goals around it depending on the status navigation returned, also resets
	 * the tree for receding modes
	 */
	void updateCurrentGoal();
	/**
	 * Timer callback for setting exploration to finished
	 * @param event
	 */
	void explorationFinishedTimerCallback(const ros::TimerEvent &event);

	bool requestGoal(rrt_nbv_exploration_msgs::RequestGoal::Request &req,
			rrt_nbv_exploration_msgs::RequestGoal::Response &res);

	bool updateCurrentGoal(
			rrt_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
			rrt_nbv_exploration_msgs::UpdateCurrentGoal::Response &res);

	bool setRrtState(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);

	bool getRrtState(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);

	bool resetRrtState(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
};
}
