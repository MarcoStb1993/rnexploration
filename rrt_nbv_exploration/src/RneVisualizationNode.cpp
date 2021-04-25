#include "ros/ros.h"
#include "rrt_nbv_exploration/RneVisualizer.h"

boost::shared_ptr<rrt_nbv_exploration::RneVisualizer> rne_visualizer;

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneVisualizationNode");
	rne_visualizer.reset(new rrt_nbv_exploration::RneVisualizer());
	ros::spin();
	rne_visualizer.reset();
	return 0;
}
