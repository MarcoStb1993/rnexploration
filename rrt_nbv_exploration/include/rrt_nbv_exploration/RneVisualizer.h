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

private:
	ros::NodeHandle _nh;
	ros::Publisher _rrt_tree_visualization_pub;
	ros::Publisher _rrt_frontier_visualization_pub;
	ros::Publisher _rrt_tree_text_info_visualization_pub;
	ros::Subscriber _rrt_tree_sub;

	/**
	 * @brief Visualization function that publishes the RRT-visualization in the topic "rrt_tree_visualization_marker" and is called when receiving new input from topic "rrt_tree"
	 * @param Received message from topic "rrt_tree"
	 */
	void visualizeRrtTree(const rrt_nbv_exploration_msgs::Tree::ConstPtr &rrt);

	/**
	 * @brief Initializes the message for topic "rrt_tree_visualization_marker"
	 * @param Message for node points
	 * @param Message for edges
	 */
	void initializeVisualization(visualization_msgs::Marker &_node_points,
			visualization_msgs::Marker &_frontier_points,
			visualization_msgs::Marker &_edge_line_list);

	/**
	 * @brief Adds info text for each node to the visualization consisting of it's number and gain
	 * @param Reference to message to populate with info text
	 * @param Position of the particular node
	 * @param Number of the node
	 * @param Gain of the node
	 */
	void addInfoTextVisualization(
			visualization_msgs::MarkerArray &_node_info_texts,
			const geometry_msgs::Point node_position, int node, double gain);

	/**
	 * @brief Return the color for the given node depending on its status and gain
	 * @param Node to determine color for
	 */
	std_msgs::ColorRGBA getColor(const rrt_nbv_exploration_msgs::Node &node);
};
}
