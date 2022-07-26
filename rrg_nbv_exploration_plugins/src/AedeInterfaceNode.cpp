/*
 * AedeInterfaceNode.cpp
 *
 *  Created on: Jul 26, 2022
 *      Author: marco
 */

#include "ros/ros.h"

#include <rrg_nbv_exploration_plugins/AedeInterface.h>

boost::shared_ptr<rne::AedeInterface> aede_interface;

void loopCallback(const ros::TimerEvent&) {
	aede_interface->updatePosition();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aedeInterface");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 1.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	aede_interface.reset(
			new rne::AedeInterface());
	ros::spin();
	aede_interface.reset();
	return 0;
}





