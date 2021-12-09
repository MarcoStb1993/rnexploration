#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include <rrg_nbv_exploration_msgs/Graph.h>
#include <rrg_nbv_exploration_msgs/Node.h>
#include <rrg_nbv_exploration_msgs/Edge.h>
#include <rrg_nbv_exploration_msgs/NodeToUpdate.h>
#include <rrg_nbv_exploration_msgs/BestAndCurrentNode.h>
#include <rrg_nbv_exploration_msgs/RequestGoal.h>
#include <rrg_nbv_exploration_msgs/RequestPath.h>
#include <rrg_nbv_exploration_msgs/UpdateCurrentGoal.h>
#include <limits.h>
#include <random>
#include "math.h"

#include <rrg_nbv_exploration/CollisionChecker.h>
#include <rrg_nbv_exploration/GraphPathCalculator.h>
#include <rrg_nbv_exploration/GraphSearcher.h>
#include <rrg_nbv_exploration/NodeComparator.h>

namespace rrg_nbv_exploration {

/**
 * @brief The GraphConstructor class constructs a RRG at a given seed position and publishes it in the rrg topic
 */
class GraphConstructor {
public:
	/**
	 * @brief Constructor that places the graph's seed at (0,0,0)
	 */
	GraphConstructor();
	~GraphConstructor();
	/**
	 * @brief Initialization for ROS and graph GraphConstructor, called by the constructors
	 * @param Seed position for the tree
	 */
	void initialization(geometry_msgs::Point seed = geometry_msgs::Point());
	/**
	 * @brief Starts the tree GraphConstructor when stopped
	 */
	void startRrgConstruction();
	/**
	 * @brief Stops the GraphConstructor when running
	 */
	void stopRrgConstruction();
	/**
	 * @brief GraphConstructor main function, publishes it in topic rrg
	 * @param kd-tree for faster nearest neighbour queries
	 */
	void runRrgConstruction();

private:
	ros::NodeHandle _nh;

	ros::Publisher _rrg_publisher;
	ros::Publisher _best_and_current_goal_publisher;
	ros::Publisher _node_to_update_publisher;
	ros::Subscriber _updated_node_subscriber;
	ros::Subscriber _octomap_sub;
	ros::ServiceServer _request_goal_service;
	ros::ServiceServer _request_path_service;
	ros::ServiceServer _update_current_goal_service;
	ros::ServiceServer _set_rrg_state_service;
	ros::ServiceServer _get_rrg_state_service;
	ros::ServiceServer _reset_rrg_state_service;
	ros::Timer _exploration_finished_timer;

	std::default_random_engine _generator;
	std::shared_ptr<octomap::AbstractOcTree> _abstract_octree;
	std::shared_ptr<octomap::OcTree> _octree;
	/**
	 * @brief Helper class for checking if a path between two nodes is collision free
	 */
	std::shared_ptr<CollisionChecker> _collision_checker;
	/**
	 * @brief Helper class for kd-tree GraphConstructor and nearest neighbour search
	 */
	std::shared_ptr<GraphSearcher> _graph_searcher;
	/**
	 * @brief Helper class for calculating a path between two nodes in the graph
	 */
	std::shared_ptr<GraphPathCalculator> _graph_path_calculator;
	/**
	 * @brief Helper class for calculating the gain cost ratio of a node and sorting them ordered by this ratio
	 */
	std::shared_ptr<NodeComparator> _node_comparator;
	/**
	 * @brief Current graph being built as a RRG
	 */
	rrg_nbv_exploration_msgs::Graph _rrg;
	/**
	 * @brief State of the GraphConstructor (running or stopped)
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
	 * @brief List of nodes (their position in the graph's node list) that require their gain to be calculated
	 */
	std::list<int> _nodes_to_update;
	/**
	 * @brief The node that is currently being pursued as a navigation goal
	 */
	int _current_goal_node;
	/**
	 * @brief Previously visited goal node
	 */
	int _last_goal_node;
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
	 * @brief Previous recorded robot position
	 */
	geometry_msgs::Point _last_robot_pos;
	/**
	 * @brief Index of previously updated node
	 */
	int _last_updated_node;
	/**
	 * @brief Time until exploration counts as finished because no new nodes were placed
	 */
	double _exploration_finished_timer_duration;
	/**
	 * @brief If the robot already moved past a neighbor node towards the current goal
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
	 * @brief If sorting the list of nodes to update is required
	 */
	bool _sort_nodes_to_update;
	/**
	 * @brief Grid map cell edge length in m
	 */
	double _grid_map_resolution;
	/**
	 * @brief If > 0 samples additional nodes in the given radius around the robot
	 */
	double _local_sampling_radius;
	/**
	 * @brief Squared distance to a node in m to count it as nearest node
	 */
	double _nearest_node_tolerance_squared;
	/**
	 * @brief Maximum number of consecutive failed goals before exploration is cancelled
	 */
	int _max_consecutive_failed_goals;
	/**
	 * @brief Actual number of consecutive failed goals
	 */
	int _consecutive_failed_goals;

	/**
	 * @brief Initialize the RRG with a root node at seed, initialize helper classes and nodes ordered by gain list with root node
	 * @param Seed position for RRG at which the root node is placed
	 * @return If initialization was successful
	 */
	bool initRrg(const geometry_msgs::Point &seed);
	/**
	 * @brief Samples new nodes and tries to connect them to the graph
	 * @param If the new nodes should be sampled locally around the robot or within map dimensions
	 * @param If the paths of possibly connected nodes should be updated
	 */
	void expandGraph(bool local, bool updatePaths);
	/**
	 * @brief Randomly samples a point from within the map dimensions
	 * @param Reference to a point that is filled with randomly sampled x and y coordinates
	 */
	void samplePoint(geometry_msgs::Point &rand_sample);
	/**
	 * @brief Randomly samples a point from within a circle around the given center
	 * @param Reference to a point that is filled with randomly sampled x and y coordinates
	 * @return If the point is inside map dimensions
	 */
	bool samplePointLocally(geometry_msgs::Point &rand_sample,
			geometry_msgs::Point center);
	/**
	 * @brief Round the given points coordinates to be in the middle of a grid cell (necessary for collision checking)
	 * @param Reference to a point which position is aligned
	 */
	void alignPointToGridMap(geometry_msgs::Point &rand_sample);
	/**
	 * @brief Tries to connect a randomly sampled point to the nearest neighbors inside the max radius in the existing graph
	 * @param Randomly sampled point
	 * @param List of nodes to connect point with and their respective squared distances
	 * @param If the paths of possibly connected nodes should be updated
	 */
	void connectNewNode(geometry_msgs::Point rand_sample,
			std::vector<std::pair<int, double>> nodes, bool updatePaths);
	/**
	 * @brief Check if there is a current goal, if there are still nodes to be explored and select a new goal if required and possible
	 */
	void checkCurrentGoal();
	/**
	 * @brief Update the PRM message variable holding the index of the node currently nearest to the robot
	 * @return Returns true if a new node is nearest to the robot
	 */
	bool determineNearestNodeToRobot();
	/**
	 * @brief Publish the node that currently has the best gain-cost-ratio
	 */
	void publishNodeWithBestGain();
	/**
	 * @brief Sorts the nodes which gain needs to be (re)calculated by their distance to the robot (closest first)
	 */
	void sortNodesToUpdateByDistanceToRobot();
	/**
	 * @brief Compares the distances to the robot of two nodes
	 * @param Node index of the first node
	 * @param Node index of the second node
	 * @return Returns true if the first node's distance to the robot is smaller than the second
	 */
	bool compareNodeDistancesToRobot(const int &node_one, const int &node_two);
	/**
	 * @brief Publish a node which gain needs to be (re)calculated
	 */
	void publishNodeToUpdate();
	/**
	 * @brief Updates all nodes' gains in the doubled sensor range around the specified center node
	 * @param Node in the center of the update circle
	 */
	void updateNodes(geometry_msgs::Point center_node);
	/**
	 * @brief Callback for subscriber to "updated_node" topic which refreshes the list of nodes to update
	 * @param Node which gain was updated
	 */
	void updatedNodeCallback(
			const rrg_nbv_exploration_msgs::Node::ConstPtr &updated_node);
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
	 * @brief Updates the current goal and the goals around it depending on the status navigation returned
	 */
	void updateCurrentGoal();
	/**
	 * Timer callback for setting exploration to finished
	 * @param event
	 */
	void explorationFinishedTimerCallback(const ros::TimerEvent &event);

	bool requestGoal(rrg_nbv_exploration_msgs::RequestGoal::Request &req,
			rrg_nbv_exploration_msgs::RequestGoal::Response &res);

	bool requestPath(rrg_nbv_exploration_msgs::RequestPath::Request &req,
			rrg_nbv_exploration_msgs::RequestPath::Response &res);

	bool updateCurrentGoal(
			rrg_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
			rrg_nbv_exploration_msgs::UpdateCurrentGoal::Response &res);

	bool setRrgState(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);

	bool getRrgState(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);

	bool resetRrgState(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
};
}
