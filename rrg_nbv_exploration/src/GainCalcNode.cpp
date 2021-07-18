#include "ros/ros.h"
#include <rrg_nbv_exploration/GainCalculator.h>

boost::shared_ptr<rrg_nbv_exploration::GainCalculator> gain_calculator;

int main(int argc, char **argv) {
	ros::init(argc, argv, "gainCalcNode");
	gain_calculator.reset(new rrg_nbv_exploration::GainCalculator());
	gain_calculator->precalculateGainPolls();
	ros::spin();
	gain_calculator.reset();
	return 0;
}

