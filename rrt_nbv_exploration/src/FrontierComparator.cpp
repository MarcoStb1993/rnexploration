/*
 * NodeComparator.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#include <rrt_nbv_exploration/FrontierComparator.h>

namespace rrt_nbv_exploration {

FrontierComparator::FrontierComparator() {
	// TODO Auto-generated constructor stub

}

FrontierComparator::~FrontierComparator() {
	// TODO Auto-generated destructor stub
}

void FrontierComparator::initialization() {
	_sort_list = false;
	_robot_moved = false;
	_frontiers_ordered_by_gcr.clear();
}

void FrontierComparator::maintainList(rrt_nbv_exploration_msgs::Frontiers &frontiers) {
	if (_sort_list) {
		calculateGainCostRatios(frontiers);
		sortByGain(frontiers);
		_robot_moved = false;
	}
}

void FrontierComparator::clear() {
	_frontiers_ordered_by_gcr.clear();
	_sort_list = false;
}

void FrontierComparator::addFrontier(int frontier) {
	_frontiers_ordered_by_gcr.emplace_back(frontier);
	_sort_list = true;
}

void FrontierComparator::removeFrontier(int frontier) {
	_frontiers_ordered_by_gcr.remove_if([frontier](CompareStruct n) {
		return n.frontier == frontier;
	});
}

int FrontierComparator::getBestFrontier() {
	return _frontiers_ordered_by_gcr.front().frontier;
}

int FrontierComparator::getListSize() {
	return _frontiers_ordered_by_gcr.size();
}

bool FrontierComparator::isEmpty() {
	return _frontiers_ordered_by_gcr.empty();
}

void FrontierComparator::robotMoved() {
	_robot_moved = true;
	_sort_list = true;
}

void FrontierComparator::setSortList() {
	_sort_list = true;
}

void FrontierComparator::sortByGain(rrt_nbv_exploration_msgs::Frontiers &frontiers) {
	_frontiers_ordered_by_gcr.sort(
			[this](CompareStruct frontier_one, CompareStruct frontier_two) {
				return compareFrontierByRatios(frontier_one, frontier_two);
			});
	_sort_list = false;
}

void FrontierComparator::calculateGainCostRatios(
		rrt_nbv_exploration_msgs::Frontiers &frontiers) {
	for (auto &frontier : _frontiers_ordered_by_gcr) {
		if (frontier.gain_cost_ratio == 0 || _robot_moved) {
			frontier.gain_cost_ratio =
					frontiers.frontiers[frontier.frontier].gain
							* exp(
									-1
											* frontiers.frontiers[frontier.frontier].distanceToRobot);
			if (frontiers.frontiers[frontier.frontier].gain == -1) //if gain=-1 the above calculation prefers frontiers further away, reverse this effect
				frontier.gain_cost_ratio = -frontier.gain_cost_ratio - 1;
		}
	}
}

double FrontierComparator::getFrontierGainCostRatio(int frontier) {
	auto it = std::find_if(_frontiers_ordered_by_gcr.begin(),
			_frontiers_ordered_by_gcr.end(), [frontier](CompareStruct n) {
				return n.frontier == frontier;
			});
	if (it != _frontiers_ordered_by_gcr.end())
		return it->gain_cost_ratio;
	else
		return 0;
}

bool FrontierComparator::compareFrontierByRatios(const CompareStruct &frontier_one,
		const CompareStruct &frontier_two) {
	return frontier_one.gain_cost_ratio >= frontier_two.gain_cost_ratio;
}

} /* namespace rrt_nbv_exploration */

