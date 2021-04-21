#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "rrt_nbv_exploration_msgs/Graph.h"
#include "rrt_nbv_exploration_msgs/Node.h"
#include "stdlib.h"
#include "sstream"

namespace rrt_nbv_exploration {
/**
 * @brief The RneVisualizer class takes the RRG topic and creates spheres and lines to depict the node as markers in RViz
 */
class RneVisualizer {
public:
	/**
	 * @brief Constructor that initializes ROS and the topic to be published
	 */
	RneVisualizer();
	~RneVisualizer();
	/**
	 * @brief Initializes the message for topic "rrg_tree_visualization_marker"
	 */
	void initializeVisualization();

private:
	ros::NodeHandle _nh;
	ros::Publisher _rrg_visualization_pub;
	ros::Publisher _rrg_text_info_visualization_pub;
	ros::Subscriber _rrg_tree_sub;
	/**
	 * @brief Visualization of the nodes in the graph as cubes
	 */
	visualization_msgs::Marker _node_points;
	/**
	 * @brief Visualization of the edges in the graph as lines
	 */
	visualization_msgs::Marker _edge_line_list;
	/**
	 * @brief Visualization of the node's number and gain as text
	 */
	visualization_msgs::MarkerArray _node_info_texts;

	/**
	 * @brief Visualization function that publishes the RRG-visualization in the topic "rrg_visualization_marker" and is called when receiving new input from topic "prm"
	 * @param Received message from topic "rrg"
	 */
	void visualizeRrgGraph(const rrt_nbv_exploration_msgs::Graph::ConstPtr& rrg);
	/**
	 * @brief Adds info text for each node to the visualization consisting of it's number and gain
	 * @param Position of the particular node
	 * @param Number of the node
	 * @param Gain of the node
	 */
	void addInfoTextVisualization(const geometry_msgs::Point node_position,
			int node, double gain);
};
}
