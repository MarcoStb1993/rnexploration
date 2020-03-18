#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "rrt_nbv_exploration_msgs/Tree.h"
#include "rrt_nbv_exploration_msgs/Node.h"
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
	ros::Publisher _rrt_tree_visualization_pub;
	ros::Publisher _rrt_tree_text_info_visualization_pub;
	ros::Subscriber _rrt_tree_sub;
	/**
	 * @brief Visualization of the nodes in the tree as cubes
	 */
	visualization_msgs::Marker _node_points;
	/**
	 * @brief Visualization of the edges in the trees as lines
	 */
	visualization_msgs::Marker _edge_line_list;
	/**
	 * @brief Visualization of the node's number and gain as text
	 */
	visualization_msgs::MarkerArray _node_info_texts;

	/**
	 * @brief Visualization function that publishes the RRT-visualization in the topic "rrt_tree_visualization_marker" and is called when receiving new input from topic "rrt_tree"
	 * @param Received message from topic "rrt_tree"
	 */
	void visualizeRrtTree(const rrt_nbv_exploration_msgs::Tree::ConstPtr& rrt);
	/**
	 * @brief Adds info text for each node to the visualization consisting of it's number and gain
	 * @param Position of the particular node
	 * @param Number of the node
	 * @param Gain of the node
	 */
	void addInfoTextVisualization(const geometry_msgs::Point node_position,
			int node, int gain);
};
}
