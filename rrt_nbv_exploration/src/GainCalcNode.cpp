#include "ros/ros.h"
#include "rrt_nbv_exploration/GainCalculator.h"

boost::shared_ptr<rrt_nbv_exploration::GainCalculator> gain_calculator;

int main(int argc, char **argv) {
	ros::init(argc, argv, "gainCalcNode");
	gain_calculator.reset(new rrt_nbv_exploration::GainCalculator());
	gain_calculator->precalculateGainPollPoints();
	ros::spin();
	gain_calculator.reset();
	return 0;
}




