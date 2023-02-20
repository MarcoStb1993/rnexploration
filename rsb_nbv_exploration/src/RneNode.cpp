#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <rsb_nbv_exploration/GraphConstructorConfig.h>
#include <rsb_nbv_exploration/GraphConstructor.h>

boost::shared_ptr<rsb_nbv_exploration::GraphConstructor> graph_constructor;

void loopCallback(const ros::TimerEvent&) {
	graph_constructor->runExploration();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneNode");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 1.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	graph_constructor.reset(new rsb_nbv_exploration::GraphConstructor());
	graph_constructor->initialization();

	dynamic_reconfigure::Server<rsb_nbv_exploration::GraphConstructorConfig> server;
	dynamic_reconfigure::Server<rsb_nbv_exploration::GraphConstructorConfig>::CallbackType f;

	f = boost::bind(
			&rsb_nbv_exploration::GraphConstructor::dynamicReconfigureCallback,
			&*graph_constructor, _1, _2);
	server.setCallback(f);

	ros::spin();
	graph_constructor.reset();
	return 0;
}
