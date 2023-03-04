/*
 * AedeInterfaceNode.cpp
 *
 *  Created on: Jul 26, 2022
 *      Author: marco
 */

#include "ros/ros.h"

#include <std_msgs/Bool.h>
#include <rsb_nbv_exploration_plugins/AedeInterface.h>

boost::shared_ptr<rne::AedeInterface> aede_interface;
ros::Subscriber rrg_sub;

void loopCallback(const ros::TimerEvent&) {
	aede_interface->updatePosition();
}

void stateCallback(const std_msgs::Bool::ConstPtr &state) {
	if (state->data) {
		aede_interface->startExploration();
	} else {
		aede_interface->stopExploration();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "aedeInterface");
	aede_interface.reset(new rne::AedeInterface());

	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 1.0);
	ros::NodeHandle nh("rne");
	rrg_sub = nh.subscribe("state", 10, stateCallback);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback, false);
	ros::spin();
	aede_interface.reset();
	return 0;
}

