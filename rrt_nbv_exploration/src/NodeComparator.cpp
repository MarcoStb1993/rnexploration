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
	int rne_mode;
	private_nh.param("rne_mode", rne_mode, (int) RneMode::classic);
	_rne_mode = static_cast<RneMode>(rne_mode);
	private_nh.param("horizon_length", _horizon_length, 3);
	_horizon_length = std::max(1, _horizon_length);	//horizon length must be at least 1

	_sort_list = false;
	_robot_moved = false;
	_nodes_ordered_by_gcr.clear();
	_nodes_ordered_by_hgcr.clear();
}

void NodeComparator::maintainList(rrt_nbv_exploration_msgs::Tree &rrt) {
	if (_sort_list) {
		calculateGainCostRatios(rrt);
		if (_rne_mode == RneMode::horizon)
			calculateHorizonGainCostRatios(rrt);
		sortByGain(rrt);
	}
}

void NodeComparator::clear() {
	_nodes_ordered_by_gcr.clear();
	_nodes_ordered_by_hgcr.clear();
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
	if (_rne_mode == RneMode::horizon
			&& _nodes_ordered_by_hgcr.front().gain_cost_ratio > 0) {//if no gain in horizon, get overall best node
		return _nodes_ordered_by_hgcr.front().node;
	} else {
		return _nodes_ordered_by_gcr.front().node;
	}
}

bool NodeComparator::isEmpty() {
	return _nodes_ordered_by_gcr.empty();
}

void NodeComparator::robotMoved() {
	_robot_moved = true;
	_sort_list = true;
}

void NodeComparator::sortByGain(rrt_nbv_exploration_msgs::Tree &rrt) {
	_nodes_ordered_by_gcr.sort(
			[this](CompareStruct node_one, CompareStruct node_two) {
				return compareNodeByRatios(node_one, node_two);
			});
	if (_rne_mode == RneMode::horizon)
		_nodes_ordered_by_hgcr.sort(
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
		}
	}
}

void NodeComparator::calculateHorizonGainCostRatios(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	_nodes_ordered_by_hgcr.clear();
	std::vector<int> start_nodes(rrt.nodes[rrt.nearest_node].children);
	if (rrt.nearest_node != 0) //root node has no parent
		start_nodes.push_back(rrt.nodes[rrt.nearest_node].parent);
	double nearest_node_gcr = getNodeGainCostRatio(rrt.nearest_node);
	if (nearest_node_gcr > 0) {	//add nearest node to list if it still has gain
		_nodes_ordered_by_hgcr.emplace_back(rrt.nearest_node, nearest_node_gcr);
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
	//initialize list of horizons to check with all children and parent of node except for nearest to robot
	std::stack<HorizonStruct> horizon_list;
	do {
		//ros::Duration(3).sleep();
		double current_hgcr = getCurrentHorizonGainCostRatio(horizon_list,
				current_node);
		bool next_node = false;
		if (horizon_list.size() < _horizon_length) { //try to add new nodes if horizon is not reached
			std::vector<int> node_list;
			if (current_node != 0
					&& last_node != rrt.nodes[current_node].parent) { //root node has no parent
				node_list.push_back(rrt.nodes[current_node].parent);
			}
			for (auto child : rrt.nodes[current_node].children) {
				if (last_node != child) {
					node_list.push_back(child);
				}
			}
			if (node_list.empty()) { //last node in branch, go to next node
				best_hgcr = std::max(current_hgcr, best_hgcr);
				next_node = true;
			} else { //additional layers to add
				horizon_list.emplace(current_node, node_list, current_hgcr);
				last_node = current_node;
				current_node = horizon_list.top().nodes.top();
				current_horizon.push_back(current_node);
			}
		} else { //horizon reached, go to next node
			best_hgcr = std::max(current_hgcr, best_hgcr);
			next_node = true;
		}
		if (best_hgcr == current_hgcr)
			best_horizon = current_horizon;
		while (!horizon_list.empty() && next_node) { //pop stacks until next node is found
			horizon_list.top().nodes.pop();
			current_horizon.pop_back();
			if (!horizon_list.top().nodes.empty()) {
				last_node = horizon_list.top().previous_node;
				current_node = horizon_list.top().nodes.top();
				current_horizon.push_back(current_node);
				next_node = false;
			} else {
				horizon_list.pop();
			}
		}
	} while (!horizon_list.empty());
	for (auto i : best_horizon) { //find first unexplored node in horizon
		if (rrt.nodes[i].status != rrt_nbv_exploration_msgs::Node::EXPLORED) {
			if (i != node) {
				node = i;
			}
			break;
		}
	}
	return CompareStruct(node, best_hgcr);;
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

