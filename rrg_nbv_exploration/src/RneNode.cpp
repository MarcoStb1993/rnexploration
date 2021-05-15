#include "ros/ros.h"

#include <rrg_nbv_exploration/GraphConstructor.h>

boost::shared_ptr<rrg_nbv_exploration::GraphConstructor> graph_constructor;

void loopCallback(const ros::TimerEvent&) {
	graph_constructor->runRrgConstruction();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneNode");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 20.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	graph_constructor.reset(new rrg_nbv_exploration::GraphConstructor());
	graph_constructor->initialization();
	ros::spin();
	graph_constructor.reset();
	return 0;
}
