#include "ros/ros.h"
#include "rrt_nbv_exploration/RneVisualizer.h"

boost::shared_ptr<rrt_nbv_exploration::RneVisualizer> rne_visualizer;

//void loopCallback(const ros::TimerEvent&) {}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneVisualizationNode");
	//ros::NodeHandle private_nh("~");
	//double loop_rate;
	//private_nh.param("update_frequency", loop_rate, 20.0);
	//ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),	loopCallback);
	rne_visualizer.reset(new rrt_nbv_exploration::RneVisualizer());
	rne_visualizer->initialize_visualization();
	ros::spin();
	rne_visualizer.reset();
	return 0;
}
