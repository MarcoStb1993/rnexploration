/*
 * FrontierComparator.cpp
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
	ros::NodeHandle private_nh("~");
	private_nh.param("cost_factor", _cost_factor, 0.5);

	_nodes_ordered_by_gcr.clear();
}

void FrontierComparator::maintainList(rrt_nbv_exploration_msgs::Tree &rrt) {
	calculateGainCostRatios(rrt);
	sortByGain(rrt);
}

void FrontierComparator::clear() {
	_nodes_ordered_by_gcr.clear();
}

void FrontierComparator::addNode(int node) {
	_nodes_ordered_by_gcr.emplace_back(node);
}

void FrontierComparator::removeNode(int node) {
	_nodes_ordered_by_gcr.remove_if([node](FrontierCompareStruct n) {
		return n.node == node;
	});
}

int FrontierComparator::getBestNode() {
	return _nodes_ordered_by_gcr.front().node;
}

int FrontierComparator::getListSize() {
	return _nodes_ordered_by_gcr.size();
}

bool FrontierComparator::isEmpty() {
	return _nodes_ordered_by_gcr.empty();
}

void FrontierComparator::sortByGain(rrt_nbv_exploration_msgs::Tree &rrt) {
	_nodes_ordered_by_gcr.sort(
			[this](FrontierCompareStruct node_one,
					FrontierCompareStruct node_two) {
				return compareNodeByRatios(node_one, node_two);
			});
}

void FrontierComparator::calculateGainCostRatios(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &node : _nodes_ordered_by_gcr) {
		if (node.gain_cost_ratio == 0) {
			node.gain_cost_ratio = rrt.frontiers[node.node].gain
					* exp(
							-1.0 * _cost_factor
									* rrt.frontiers[node.node].distanceToRobot);
			if (rrt.frontiers[node.node].gain == -1) //if gain=-1 the above calculation prefers nodes further away, reverse this effect
				node.gain_cost_ratio = -node.gain_cost_ratio - 1;
		}
	}
}

bool FrontierComparator::compareNodeByRatios(
		const FrontierCompareStruct &node_one,
		const FrontierCompareStruct &node_two) {
	return node_one.gain_cost_ratio >= node_two.gain_cost_ratio;
}

} /* namespace rrt_nbv_exploration */

