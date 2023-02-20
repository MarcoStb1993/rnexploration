#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <rsb_nbv_exploration/GainCalculatorConfig.h>
#include <rsb_nbv_exploration/GainCalculator.h>

boost::shared_ptr<rsb_nbv_exploration::GainCalculator> gain_calculator;

int main(int argc, char **argv) {
	ros::init(argc, argv, "gainCalcNode");
	gain_calculator.reset(new rsb_nbv_exploration::GainCalculator());
	gain_calculator->precalculateGainPolls();

	dynamic_reconfigure::Server<rsb_nbv_exploration::GainCalculatorConfig> server;
	dynamic_reconfigure::Server<rsb_nbv_exploration::GainCalculatorConfig>::CallbackType f;

	f = boost::bind(&rsb_nbv_exploration::GainCalculator::dynamicReconfigureCallback,
			&*gain_calculator, _1, _2);
	server.setCallback(f);

	ros::spin();
	gain_calculator.reset();
	return 0;
}

