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

	_sort_list = false;
	_robot_moved = false;
	_nodes_ordered_by_gain.clear();
}

void NodeComparator::maintainList(rrt_nbv_exploration_msgs::Tree &rrt) {
	if (_sort_list) {
		calculatePathDistances(rrt);
		if (_rne_mode == RneMode::horizon)
			calculateHorizonGainCostRatio(rrt);
		else
			calculateGainCostRatio(rrt);
		sortByGain(rrt);
//		for (auto it : _nodes_ordered_by_gain) {
//			ROS_INFO_STREAM(
//					std::setprecision(2) << "Node " << it.node << " with gcr: " << it.gain_cost_ratio << ", distance: " << it.distance_to_robot <<" and status: " << (int) rrt.nodes[it.node].status);
//		}
	}
}

void NodeComparator::clear() {
	_nodes_ordered_by_gain.clear();
	_sort_list = false;
}

void NodeComparator::addNode(int node) {
	_nodes_ordered_by_gain.push_back(CompareStruct(node));
	_sort_list = true;
}

void NodeComparator::removeNode(int node) {
	_nodes_ordered_by_gain.remove_if([node](CompareStruct n) {
		return n.node == node;
	});
}

int NodeComparator::getBestNode() {
	return _nodes_ordered_by_gain.front().node;
}

bool NodeComparator::isEmpty() {
	return _nodes_ordered_by_gain.empty();
}

void NodeComparator::robotMoved() {
	_robot_moved = true;
	_sort_list = true;
}

void NodeComparator::sortByGain(rrt_nbv_exploration_msgs::Tree &rrt) {
	if (_rne_mode == RneMode::horizon)
		_nodes_ordered_by_gain.sort(
				[this](CompareStruct node_one, CompareStruct node_two) {
					return compareNodeByHorizonRatios(node_one, node_two);
				});
	else
		//classic mode
		_nodes_ordered_by_gain.sort(
				[this](CompareStruct node_one, CompareStruct node_two) {
					return compareNodeByRatios(node_one, node_two);
				});
	_sort_list = false;
}

void NodeComparator::calculatePathDistances(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	for (auto &node : _nodes_ordered_by_gain) {
		if (_robot_moved || node.distance_to_robot < 0) {
			node.distance_to_robot = calculatePathDistance(rrt,
					rrt.nodes[node.node].pathToRobot);
			node.gain_cost_ratio = 0.0;
			node.horizon_gain_cost_ratio = 0.0;
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

void NodeComparator::calculateGainCostRatio(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	//TODO: Calculate for each node if gcr = 0
	for (auto &node : _nodes_ordered_by_gain) {
		if (node.gain_cost_ratio == 0) {
			node.gain_cost_ratio = rrt.nodes[node.node].gain
					* exp(-1 * node.distance_to_robot);
		}
	}
}

void NodeComparator::calculateHorizonGainCostRatio(
		rrt_nbv_exploration_msgs::Tree &rrt) {
	//TODO: Calculate for each node if hgcr = 0
}

bool NodeComparator::compareNodeByRatios(const CompareStruct &node_one,
		const CompareStruct &node_two) {
	return node_one.gain_cost_ratio >= node_two.gain_cost_ratio;
}

bool NodeComparator::compareNodeByHorizonRatios(const CompareStruct &node_one,
		const CompareStruct &node_two) {
	return false;
}

} /* namespace rrt_nbv_exploration */

