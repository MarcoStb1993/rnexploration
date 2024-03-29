#include "ros/ros.h"

#include <rsb_nbv_exploration_plugins/RneServiceProvider.h>

boost::shared_ptr<rsm::RneServiceProvider> service_provider;

void loopCallback(const ros::TimerEvent&) {
	service_provider->publishTopics();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneServiceProvider");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 20.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	service_provider.reset(
			new rsm::RneServiceProvider());
	ros::spin();
	service_provider.reset();
	return 0;
}
