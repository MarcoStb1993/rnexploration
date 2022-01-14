#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "rrg_nbv_exploration_msgs/Graph.h"
#include "rrg_nbv_exploration_msgs/Node.h"
#include "stdlib.h"
#include "sstream"

namespace rrg_nbv_exploration {
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

private:
	ros::NodeHandle _nh;
	ros::Publisher _rrg_visualization_pub;
	ros::Publisher _rrg_text_info_visualization_pub;
	ros::Subscriber _rrg_sub;
	/**
	 * Last time at which the graph info was published
	 */
	ros::Time _last_info_publish;
	/**
	 * Interval time at which the graph info should be published (in s)
	 */
	double _info_interval;
	/**
	 * Node count in the RRG
	 */
	int _last_rrg_node_count;

	/**
	 * @brief Visualization function that publishes the RRG-visualization in the topic "rrg_vis" and is called when receiving new input from topic "rrg"
	 * @param Received message from topic "rrg"
	 */
	void visualizeRrgGraph(
			const rrg_nbv_exploration_msgs::Graph::ConstPtr &rrg);

	/**
	 * @brief Initializes the message for topic "rrg_vis"
	 * @param Message for node points
	 * @param Message for edges
	 */
	void initializeVisualization(visualization_msgs::Marker &_node_points,
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
			const geometry_msgs::Point node_position, int node, double reward_function);

	/**
	 * @brief Clear the info text markers if the graph is being rebuilt
	 */
	void clearInfoText();

	/**
	 * @brief Return the color for the given node depending on its status and gain
	 * @param Node to determine color for
	 */
	std_msgs::ColorRGBA getColor(const rrg_nbv_exploration_msgs::Node &node);
};
}
