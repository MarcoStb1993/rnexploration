#include "ros/ros.h"

#include <rrg_nbv_exploration/RneVisualizer.h>

boost::shared_ptr<rrg_nbv_exploration::RneVisualizer> rne_visualizer;

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneVisualizationNode");
	rne_visualizer.reset(new rrg_nbv_exploration::RneVisualizer());
	ros::spin();
	rne_visualizer.reset();
	return 0;
}
