/*
 * AedeInterfaceNode.cpp
 *
 *  Created on: Jul 26, 2022
 *      Author: marco
 */

#include "ros/ros.h"

#include <rrg_nbv_exploration_plugins/AedeInterface.h>
#include <rrg_nbv_exploration_msgs/Graph.h>

boost::shared_ptr<rne::AedeInterface> aede_interface;
ros::Timer loop_timer;
ros::Subscriber rrg_sub;

void loopCallback(const ros::TimerEvent&) {
	aede_interface->updatePosition();
}

void rrgCallback(const rrg_nbv_exploration_msgs::Graph::ConstPtr &rrg) {
	if (rrg->node_counter) {
		loop_timer.start();
		rrg_sub.shutdown();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aedeInterface");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 1.0);
	aede_interface.reset(new rne::AedeInterface());
	ros::NodeHandle nh("rne");
	rrg_sub = nh.subscribe("rrg", 10, rrgCallback);
	loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback, false, false);
	ros::spin();
	aede_interface.reset();
	return 0;
}

