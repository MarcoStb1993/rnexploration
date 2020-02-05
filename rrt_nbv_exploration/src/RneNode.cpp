#include "ros/ros.h"
#include "rrt_nbv_exploration/TreeConstructor.h"

boost::shared_ptr<rrt_nbv_exploration::TreeConstructor> tree_constructor;

void loopCallback(const ros::TimerEvent&) {
	//tree_constructor->publishTopics();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rneNode");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 20.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	tree_constructor.reset(new rrt_nbv_exploration::TreeConstructor());
	ros::spin();
	tree_constructor.reset();
	return 0;
}
