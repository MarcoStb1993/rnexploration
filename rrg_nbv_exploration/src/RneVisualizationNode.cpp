#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <rrg_nbv_exploration/RneVisualizer.h>
#include <rrg_nbv_exploration/RneVisualizerConfig.h>

boost::shared_ptr<rrg_nbv_exploration::RneVisualizer> rne_visualizer;

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneVisualizationNode");
	rne_visualizer.reset(new rrg_nbv_exploration::RneVisualizer());

	dynamic_reconfigure::Server<rrg_nbv_exploration::RneVisualizerConfig> server;
		dynamic_reconfigure::Server<rrg_nbv_exploration::RneVisualizerConfig>::CallbackType f;

		f = boost::bind(
				&rrg_nbv_exploration::RneVisualizer::dynamicReconfigureCallback,
				&*rne_visualizer, _1, _2);
		server.setCallback(f);

	ros::spin();
	rne_visualizer.reset();
	return 0;
}
