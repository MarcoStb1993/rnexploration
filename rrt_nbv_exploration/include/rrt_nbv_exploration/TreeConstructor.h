#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.h"
#include <rrt_nbv_exploration_msgs/Tree.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include <rrt_nbv_exploration_msgs/NodeList.h>
#include <rrt_nbv_exploration_msgs/BestAndCurrentNode.h>
#include <rrt_nbv_exploration_msgs/RequestGoal.h>
#include <rrt_nbv_exploration_msgs/RequestPath.h>
#include <rrt_nbv_exploration_msgs/UpdateCurrentGoal.h>
#include <limits.h>
#include <random>
#include "math.h"
#include "rrt_nbv_exploration/CollisionChecker.h"
#include "rrt_nbv_exploration/TreePathCalculator.h"
#include "rrt_nbv_exploration/TreeSearcher.h"

namespace rrt_nbv_exploration
{
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

    ros::Publisher _rrt_publisher;
    ros::Publisher _best_and_current_goal_publisher;
    ros::Publisher _node_to_update_publisher;
    ros::Subscriber _updated_node_subscriber;
    ros::Subscriber _octomap_sub;
    ros::ServiceServer _request_goal_service;
    ros::ServiceServer _request_path_service;
    ros::ServiceServer _update_current_goal_service;
    ros::ServiceServer _set_rrt_state_service;
    ros::ServiceServer _get_rrt_state_service;
    ros::ServiceServer _reset_rrt_state_service;
	ros::Timer _exploration_finished_timer;

    std::default_random_engine _generator;
    std::shared_ptr<octomap::AbstractOcTree> _abstract_octree;
    std::shared_ptr<octomap::OcTree> _octree;
    /**
     * @brief Helper class for checking if a path between two nodes is collision free
     */
    std::shared_ptr<CollisionChecker> _collision_checker;
    /**
     * @brief Helper class for kd-tree TreeConstructor and nearest neighbour search
     */
    std::shared_ptr<TreeSearcher> _tree_searcher;
    /**
     * @brief Helper class for calculating a path between two nodes in the tree
     */
    std::shared_ptr<TreePathCalculator> _tree_path_calculator;
    /**
     * @brief Current tree being built as a RRT
     */
    rrt_nbv_exploration_msgs::Tree _rrt;
    /**
     * @brief State of the tree TreeConstructor (running or stopped)
     */
    bool _running;
    /**
     * @brief Current dimension of the 3D map as x, y and z value
     */
    double _map_dimensions[3];
    /**
     * @brief All nodes (their position in the rrt node list) ordered ascending by a gain function
     */
    std::list<int> _nodes_ordered_by_gain;
    /**
     * @brief List of nodes (their position in the rrt node list) that require their gain to be calculated
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
	 * @brief Fixed length of the tree's edges, flexible if set to -1 (TODO: wavefront if set to 0)
	 */
	double _edge_length;
	/**
	 * @brief Previous recorded robot position
	 */
	geometry_msgs::Point _last_robot_pos;
	/**
	 * @brief Index of previously updated node
	 */
	int _last_updated_node;
	/**
	 * Time until exploration counts as finished because no new nodes were placed
	 */
	double _exploration_finished_timer_duration;

    /**
     * @brief Initialize the RRT with a root node at seed, initialize helper classes and nodes ordered by gain list with root node
     */
	void initRrt(const geometry_msgs::Point& seed);
    /**
     * @brief Randomly samples a point from the map dimension and returns it
     * @return Randomly sampled point
     */
     void samplePoint(geometry_msgs::Point& rand_sample);
    /**
     * @brief Returns a new point for the tree to incorporate as node regarding the randomly sampled point and it's nearest neighbour in the tree
     * @param Randomly sampled point
     * @param Minimum squared distance calculated between a node in the tree and the randomly sampled point
     * @param Nearest node in the tree from the randomly sampled point
     */
    void placeNewNode(geometry_msgs::Point rand_sample, double min_distance, int nearest_node);
    /**
     * @brief Update the RRT message variable holding the index of the node currently nearest to the robot
     */
    void determineNearestNodeToRobot();
    /**
     * @brief Compares the two given nodes and returns true if the first node's gain function is better than the second node's
     * @param Position in the node list of the first node
     * @param Position in the node list of the second node
     * @param Returns if the first node's gain is better than the second node's gain
     */
    bool compareNodeGains(const int& node_one, const int& node_two);
    /**
     * @brief Sorts list nodes_ordered_by_gain by gain function, the node with the best gain comes first
     */
    void sortNodesByGain();
    /**
     * @brief publish_node_with_best_gain
     */
    void publishNodeWithBestGain();
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
    void updatedNodeCallback(const rrt_nbv_exploration_msgs::Node::ConstPtr& updated_node);
    /**
     * @brief Function called by subscriber to "octomap_binary" message and converts it to the octree data format for further processing
     * @param "octomap_binary" message
     */
    void convertOctomapMsgToOctree(const octomap_msgs::Octomap::ConstPtr& map_msg);
    /**
     * @brief Updates the map's dimension (x,y,z) after a new octomap was received
     */
    void updateMapDimensions();
    /**
     * @brief Updates the current goal and publishes it to navigation
     */
    void updateCurrentGoal();
	/**
	 * Timer callback for setting exploration to finished
	 * @param event
	 */
	void explorationFinishedTimerCallback(const ros::TimerEvent& event);

    bool requestGoal(rrt_nbv_exploration_msgs::RequestGoal::Request &req,
    		rrt_nbv_exploration_msgs::RequestGoal::Response &res);

	bool requestPath(rrt_nbv_exploration_msgs::RequestPath::Request &req,
			rrt_nbv_exploration_msgs::RequestPath::Response &res);

    bool updateCurrentGoal(rrt_nbv_exploration_msgs::UpdateCurrentGoal::Request &req,
        		rrt_nbv_exploration_msgs::UpdateCurrentGoal::Response &res);

    bool setRrtState(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    bool getRrtState(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool resetRrtState(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};
}
