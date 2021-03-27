/*
 * NodeComparator.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#include <rrt_nbv_exploration/NodeComparator.h>

namespace rrt_nbv_exploration {

NodeComparator::NodeComparator() {
	// TODO Auto-generated constructor stub

}

NodeComparator::~NodeComparator() {
	// TODO Auto-generated destructor stub
}

void NodeComparator::initialization() {
	ros::NodeHandle private_nh("~");

	_sort_list = false;
	_robot_moved = false;
	_nodes_ordered_by_gcr.clear();
}

void NodeComparator::maintainList(rrt_nbv_exploration_msgs::Tree &rrt) {
	if (_sort_list) {
		calculateGainCostRatios(rrt);
		sortByGain(rrt);
	}
}

void NodeComparator::clear() {
	_nodes_ordered_by_gcr.clear();
	_sort_list = false;
}

void NodeComparator::addNode(int node) {
	_nodes_ordered_by_gcr.emplace_back(node);
	_sort_list = true;
}

void NodeComparator::removeNode(int node) {
	_nodes_ordered_by_gcr.remove_if([node](CompareStruct n) {
		return n.node == node;
	});
}

int NodeComparator::getBestNode() {
	return _nodes_ordered_by_gcr.front().node;
}

int NodeComparator::getListSize() {
	return _nodes_ordered_by_gcr.size();
}

bool NodeComparator::isEmpty() {
	return _nodes_ordered_by_gcr.empty();
}

void NodeComparator::robotMoved() {
	_robot_moved = true;
	_sort_list = true;
}

void NodeComparator::setSortList() {
	_sort_list = true;
}

void NodeComparator::sortByGain(rrt_nbv_exploration_msgs::Tree &rrt) {
	_nodes_ordered_by_gcr.sort(
			[this](CompareStruct node_one, CompareStruct node_two) {
				return compareNodeByRatios(node_one, node_two);
			});
	_sort_list = false;
}

void NodeComparator::calculateGainCostRatios(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &node : _nodes_ordered_by_gcr) {
		if (node.gain_cost_ratio == 0) {
			node.gain_cost_ratio = rrt.nodes[node.node].gain
					* exp(-1 * rrt.nodes[node.node].distanceToRobot);
			if (rrt.nodes[node.node].gain == -1) //if gain=-1 the above calculation prefers nodes further away, reverse this effect
				node.gain_cost_ratio = -node.gain_cost_ratio - 1;
		}
	}
}

double NodeComparator::getNodeGainCostRatio(int node) {
	auto it = std::find_if(_nodes_ordered_by_gcr.begin(),
			_nodes_ordered_by_gcr.end(), [node](CompareStruct n) {
				return n.node == node;
			});
	if (it != _nodes_ordered_by_gcr.end())
		return it->gain_cost_ratio;
	else
		return 0;
}

bool NodeComparator::compareNodeByRatios(const CompareStruct &node_one,
		const CompareStruct &node_two) {
	return node_one.gain_cost_ratio >= node_two.gain_cost_ratio;
}

} /* namespace rrt_nbv_exploration */

