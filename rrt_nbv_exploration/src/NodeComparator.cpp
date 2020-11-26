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
	private_nh.param("edge_length", _edge_length, 1.0);
	int rne_mode;
	private_nh.param("rne_mode", rne_mode, (int) RneMode::classic);
	_rne_mode = static_cast<RneMode>(rne_mode);
	private_nh.param("horizon_length", _horizon_length, 3);

	_sort_list = false;
	_robot_moved = false;
	_nodes_ordered_by_gcr.clear();
	_nodes_ordered_by_hgcr.clear();
}

void NodeComparator::maintainList(rrt_nbv_exploration_msgs::Tree &rrt) {
	if (_sort_list) {
		calculatePathDistances(rrt);
		calculateGainCostRatios(rrt);
		if (_rne_mode == RneMode::horizon)
			calculateHorizonGainCostRatios(rrt);
		sortByGain(rrt);
//		for (auto it : _nodes_ordered_by_gcr) {
//			ROS_INFO_STREAM(
//					std::setprecision(2) << "Node " << it.node << " with gcr: " << it.gain_cost_ratio << ", distance: " << it.distance_to_robot <<" and status: " << (int) rrt.nodes[it.node].status);
//		}
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
	if (_rne_mode == RneMode::horizon)
		return _nodes_ordered_by_hgcr.front().node;
	else
		return _nodes_ordered_by_gcr.front().node;
}

bool NodeComparator::isEmpty() {
	return _nodes_ordered_by_gcr.empty();
}

void NodeComparator::robotMoved() {
	_robot_moved = true;
	_sort_list = true;
}

void NodeComparator::sortByGain(rrt_nbv_exploration_msgs::Tree &rrt) {
	if (_rne_mode == RneMode::horizon)
		_nodes_ordered_by_hgcr.sort(
				[this](CompareStruct node_one, CompareStruct node_two) {
					return compareNodeByRatios(node_one, node_two);
				});
	else
		//classic mode
		_nodes_ordered_by_gcr.sort(
				[this](CompareStruct node_one, CompareStruct node_two) {
					return compareNodeByRatios(node_one, node_two);
				});
	_sort_list = false;
}

void NodeComparator::calculatePathDistances(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &node : _nodes_ordered_by_gcr) {
		if (_robot_moved || node.distance_to_robot < 0) {
			node.distance_to_robot = calculatePathDistance(rrt,
					rrt.nodes[node.node].pathToRobot);
			node.gain_cost_ratio = 0.0;
		}
	}
	_robot_moved = false;
}

double NodeComparator::calculatePathDistance(
		rrt_nbv_exploration_msgs::Tree &rrt, std::vector<int> path) {
	if (path.size() < 2) { // No distance to calculate
		return 0;
	}
	if (_edge_length > 0) { //Fixed edge length, distances between nodes are the same
		return ((double) path.size() - 1.0) * _edge_length;
	} else { //Variable edge length
		double distance = 0;
		for (auto &i : path) {
			if (&i != &path.back()) { //not last node in path
				if (rrt.nodes[i].parent == *(&i + 1)) { //i+1 is i's parent
					distance += rrt.nodes[i].distanceToParent;
				} else { //i is (i+1)'s parent
					distance += rrt.nodes[*(&i + 1)].distanceToParent;
				}
			}
		}
		return distance;
	}
}

void NodeComparator::calculateGainCostRatios(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &node : _nodes_ordered_by_gcr) {
		if (node.gain_cost_ratio == 0) {
			node.gain_cost_ratio = rrt.nodes[node.node].gain
					* exp(-1 * node.distance_to_robot);
		}
	}
}

void NodeComparator::calculateHorizonGainCostRatios(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	_nodes_ordered_by_hgcr.clear();
	std::vector<int> start_nodes(rrt.nodes[rrt.nearest_node].children);
	start_nodes.push_back(rrt.nodes[rrt.nearest_node].parent);
	ROS_INFO_STREAM(
			"Nearest node " << rrt.nearest_node << " has " << start_nodes.size() << " neighbors");
	for (auto &node : start_nodes) {
		_nodes_ordered_by_hgcr.push_back(
				calculateHorizonGainCostRatio(rrt, node));
	}
}

CompareStruct NodeComparator::calculateHorizonGainCostRatio(
		rrt_nbv_exploration_msgs::Tree &rrt, int node) {
	CompareStruct horizon_node(node);
	double best_hgcr;
	int current_node = node;
	int last_node = rrt.nearest_node;
	std::vector<int> current_horizon;
	//initialize list of horizons to check with all children and parent of node except for nearest to robot
	std::stack<HorizonStruct> horizon_list;
	do {
		double current_gcr = getCurrentHorizonGainCostRatio(horizon_list,
				current_node);
		bool next_node = false;
		if (horizon_list.size() < _horizon_length) { //add new nodes if horizon is not reached
			std::vector<int> node_list;
			if (last_node != rrt.nodes[current_node].parent)
				node_list.push_back(rrt.nodes[current_node].parent);
			for (auto child : rrt.nodes[current_node].children) {
				if (last_node != child)
					node_list.push_back(child);
			}
			if (node_list.empty()) {
				best_hgcr = std::max(current_gcr, best_hgcr);
				next_node = true;
			} else {
				horizon_list.emplace(last_node, node_list, current_gcr);
				last_node = current_node;
				current_node = horizon_list.top().nodes.top();
			}
		} else {
			best_hgcr = std::max(current_gcr, best_hgcr);
			next_node = true;
		}

		while (!horizon_list.empty() && next_node) {
			horizon_list.top().nodes.pop();
			if (!horizon_list.top().nodes.empty()) {
				last_node = horizon_list.top().previous_node;
				current_node = horizon_list.top().nodes.top();
				next_node = false;
			}else {
				horizon_list.pop();
			}
		}
	} while (!horizon_list.empty());
	horizon_node.gain_cost_ratio = best_hgcr;
	return horizon_node;
}

double NodeComparator::getCurrentHorizonGainCostRatio(
		std::stack<HorizonStruct> &horizon_list, int node) {
	double hgcr = 0;
	if (!horizon_list.empty())
		hgcr += horizon_list.top().gain_cost_ratio;

	auto it = std::find_if(_nodes_ordered_by_gcr.begin(),
			_nodes_ordered_by_gcr.end(), [node](CompareStruct n) {
				return n.node == node;
			});
	if (it != _nodes_ordered_by_gcr.end())
		hgcr += it->gain_cost_ratio;
	return hgcr;
}

bool NodeComparator::compareNodeByRatios(const CompareStruct &node_one,
		const CompareStruct &node_two) {
	return node_one.gain_cost_ratio >= node_two.gain_cost_ratio;
}

} /* namespace rrt_nbv_exploration */

