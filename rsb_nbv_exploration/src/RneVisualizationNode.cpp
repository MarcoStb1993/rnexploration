#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <rsb_nbv_exploration/RneVisualizerConfig.h>
#include <rsb_nbv_exploration/RneVisualizer.h>

boost::shared_ptr<rsb_nbv_exploration::RneVisualizer> rne_visualizer;

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneVisualizationNode");
	rne_visualizer.reset(new rsb_nbv_exploration::RneVisualizer());

	dynamic_reconfigure::Server<rsb_nbv_exploration::RneVisualizerConfig> server;
		dynamic_reconfigure::Server<rsb_nbv_exploration::RneVisualizerConfig>::CallbackType f;

		f = boost::bind(
				&rsb_nbv_exploration::RneVisualizer::dynamicReconfigureCallback,
				&*rne_visualizer, _1, _2);
		server.setCallback(f);

	ros::spin();
	rne_visualizer.reset();
	return 0;
}
