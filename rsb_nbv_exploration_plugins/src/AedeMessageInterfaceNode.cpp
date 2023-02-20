/*
 * AedeMessageInterfaceNode.cpp
 *
 *  Created on: Jul 31, 2022
 *      Author: marco
 */

#include "ros/ros.h"

#include <rsb_nbv_exploration_plugins/AedeMessageInterface.h>

boost::shared_ptr<rne::AedeMessageInterface> aede_message_interface;

int main(int argc, char **argv) {
	ros::init(argc, argv, "aedeMessageInterface");
	aede_message_interface.reset(new rne::AedeMessageInterface());
	ros::spin();
	aede_message_interface.reset();
	return 0;
}

