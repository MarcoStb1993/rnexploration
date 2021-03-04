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
	_nodes_ordered_by_gcr.clear();
	_nodes_ordered_by_hgcr.clear();
}

void NodeComparator::maintainList(rrt_nbv_exploration_msgs::Tree &rrt) {
	calculateGainCostRatios(rrt);
	calculateHorizonGainCostRatios(rrt);
	sortByGain(rrt);
}

void NodeComparator::clear() {
	_nodes_ordered_by_gcr.clear();
	_nodes_ordered_by_hgcr.clear();
}

void NodeComparator::addNode(int node) {
	_nodes_ordered_by_gcr.emplace_back(node);
}

void NodeComparator::removeNode(int node) {
	_nodes_ordered_by_gcr.remove_if([node](CompareStruct n) {
		return n.node == node;
	});
}

int NodeComparator::getBestNode() {
	return _nodes_ordered_by_hgcr.front().node;
}

std::vector<int> NodeComparator::getBestBranch() {
	return _nodes_ordered_by_hgcr.front().horizon;
}

int NodeComparator::getListSize() {
	return _nodes_ordered_by_gcr.size();
}

bool NodeComparator::isEmpty() {
	return _nodes_ordered_by_gcr.empty();
}

void NodeComparator::sortByGain(rrt_nbv_exploration_msgs::Tree &rrt) {
	_nodes_ordered_by_hgcr.sort(
			[this](CompareStruct node_one, CompareStruct node_two) {
				return compareNodeByRatios(node_one, node_two);
			});
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

void NodeComparator::calculateHorizonGainCostRatios(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	_nodes_ordered_by_hgcr.clear();
	std::vector<int> start_nodes(rrt.nodes[rrt.root].children);
	double root_node_gcr = getNodeGainCostRatio(rrt.root);
	if (root_node_gcr > 0) {	//add nearest node to list if it still has gain
		_nodes_ordered_by_hgcr.emplace_back(rrt.root, root_node_gcr);
	}
	for (auto &node : start_nodes) {
		_nodes_ordered_by_hgcr.push_back(
				calculateHorizonGainCostRatio(rrt, node));
	}
}

CompareStruct NodeComparator::calculateHorizonGainCostRatio(
		rrt_nbv_exploration_msgs::Tree &rrt, int node) {
	double best_hgcr = 0.0;
	std::vector<int> best_horizon;
	int current_node = node;
	int last_node = rrt.nearest_node;
	std::vector<int> current_horizon; //maintain for determining first unexplored node in horizon
	current_horizon.push_back(current_node);
	//initialize list of horizons to check with all children of node
	std::stack<HorizonStruct> horizon_list;
	do {
		double current_hgcr = getCurrentHorizonGainCostRatio(horizon_list,
				current_node);
		bool next_node = false;
		if (rrt.nodes[current_node].children.empty()) { //last node in branch, go to next node
			best_hgcr = std::max(current_hgcr, best_hgcr);
			next_node = true;
		} else { //additional layers to add
			horizon_list.emplace(current_node, rrt.nodes[current_node].children,
					current_hgcr);
			current_node = horizon_list.top().nodes.top();
			current_horizon.push_back(current_node);
		}
		if (best_hgcr == current_hgcr)
			best_horizon = current_horizon;
		while (!horizon_list.empty() && next_node) { //pop stacks until next node is found
			horizon_list.top().nodes.pop();
			current_horizon.pop_back();
			if (!horizon_list.top().nodes.empty()) {
				current_node = horizon_list.top().nodes.top();
				current_horizon.push_back(current_node);
				next_node = false;
			} else {
				horizon_list.pop();
			}
		}
	} while (!horizon_list.empty());
	return CompareStruct(node, best_hgcr, best_horizon);;
}

double NodeComparator::getCurrentHorizonGainCostRatio(
		std::stack<HorizonStruct> &horizon_list, int node) {
	double hgcr = 0;
	if (!horizon_list.empty())
		hgcr += horizon_list.top().gain_cost_ratio;
	hgcr += getNodeGainCostRatio(node);
	return hgcr;
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

