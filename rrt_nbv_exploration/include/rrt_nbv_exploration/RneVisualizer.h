#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "rrt_nbv_exploration_msgs/Frontiers.h"
#include "rrt_nbv_exploration_msgs/Frontier.h"
#include "stdlib.h"
#include "sstream"

namespace rrt_nbv_exploration {
/**
 * @brief The RneVisualizer class takes the RRT topic and creates spheres and lines to depict the tree as markers in RViz
 */
class RneVisualizer {
public:
	/**
	 * @brief Constructor that initializes ROS and the topic to be published
	 */
	RneVisualizer();
	~RneVisualizer();
	/**
	 * @brief Initializes the message for topic "rrt_tree_visualization_marker"
	 */
	void initializeVisualization();

private:
	ros::NodeHandle _nh;
	ros::Publisher _frontiers_visualization_pub;
	ros::Publisher _frontiers_text_info_visualization_pub;
	ros::Subscriber _frontiers_sub;
	/**
	 * @brief Visualization of the frontiers in the tree as cubes
	 */
	visualization_msgs::Marker _frontiers;
	/**
	 * @brief Visualization of the frontier's number and gain as text
	 */
	visualization_msgs::MarkerArray _frontier_info_texts;

	/**
	 * @brief Visualization function that publishes the RRT-visualization in the topic "rrt_tree_visualization_marker" and is called when receiving new input from topic "rrt_tree"
	 * @param Received message from topic "rrt_tree"
	 */
	void visualizeRrtTree(const rrt_nbv_exploration_msgs::Frontiers::ConstPtr& frontiers);
	/**
	 * @brief Adds info text for each frontier to the visualization consisting of it's number and gain
	 * @param Position of the particular frontier
	 * @param Number of the frontier
	 * @param Gain of the frontier
	 */
	void addInfoTextVisualization(const geometry_msgs::Point frontier_position,
			int frontier, double gain);
};
}
