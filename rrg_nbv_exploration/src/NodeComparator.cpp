/*
 * NodeComparator.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#include <rrg_nbv_exploration/NodeComparator.h>

namespace rrg_nbv_exploration {

NodeComparator::NodeComparator() {

}

NodeComparator::~NodeComparator() {
	// TODO Auto-generated destructor stub
}

void NodeComparator::initialization() {
	ros::NodeHandle private_nh("~");
	private_nh.param("gain_factor", _gain_factor, 1.0);
	private_nh.param("distance_factor", _distance_factor, 1.0);
	private_nh.param("heading_factor", _heading_factor, 1.0);
	private_nh.param("traversability_factor", _traversability_factor, 1.0);

	_sort_list = false;
	_robot_moved = false;
	_nodes_ordered_by_reward.clear();
}

void NodeComparator::maintainList(rrg_nbv_exploration_msgs::Graph &rrg) {
	if (_sort_list) {
		calculateRewardFunctions(rrg);
		sortByReward();
		_robot_moved = false;
	}
}

void NodeComparator::clear() {
	_nodes_ordered_by_reward.clear();
	_sort_list = false;
}

void NodeComparator::addNode(int node) {
	_nodes_ordered_by_reward.emplace_back(node);
	_sort_list = true;
}

void NodeComparator::removeNode(int node) {
	_nodes_ordered_by_reward.remove_if([node](CompareStruct n) {
		return n.node == node;
	});
}

int NodeComparator::getBestNode() {
	return _nodes_ordered_by_reward.front().node;
}

int NodeComparator::getListSize() {
	return _nodes_ordered_by_reward.size();
}

bool NodeComparator::isEmpty() {
	return _nodes_ordered_by_reward.empty();
}

void NodeComparator::robotMoved() {
	_robot_moved = true;
	_sort_list = true;
}

void NodeComparator::setSortList() {
	_sort_list = true;
}

void NodeComparator::sortByReward() {
	_nodes_ordered_by_reward.sort(
			[this](CompareStruct node_one, CompareStruct node_two) {
				return compareNodeByReward(node_one, node_two);
			});
	_sort_list = false;
}

void NodeComparator::calculateRewardFunctions(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	for (auto &node : _nodes_ordered_by_reward) {
		if (node.reward_function == 0 || _robot_moved) {
			node.reward_function =
					(_gain_factor * rrg.nodes[node.node].gain
							+ _distance_factor
									* (1.0
											- (rrg.nodes[node.node].distance_to_robot
													/ rrg.longest_distance_to_robot))
							+ (_traversability_factor
									* (1.0
											- rrg.nodes[node.node].traversability_cost_to_robot))
							+ (_heading_factor
									* (1.0
											- rrg.nodes[node.node].heading_change_to_robot_best_view)))
							/ 4.0;
			rrg.nodes[node.node].reward_function = node.reward_function;
		}
	}
}

double NodeComparator::getNodeRewardFunction(int node) {
	auto it = std::find_if(_nodes_ordered_by_reward.begin(),
			_nodes_ordered_by_reward.end(), [node](CompareStruct n) {
				return n.node == node;
			});
	if (it != _nodes_ordered_by_reward.end())
		return it->reward_function;
	else
		return 0;
}

bool NodeComparator::compareNodeByReward(const CompareStruct &node_one,
		const CompareStruct &node_two) {
	return node_one.reward_function >= node_two.reward_function;
}

void NodeComparator::dynamicReconfigureCallback(
		rrg_nbv_exploration::GraphConstructorConfig &config, uint32_t level) {
	_gain_factor = config.gain_factor;
	_distance_factor = config.distance_factor;
	_traversability_factor = config.traversability_factor;
	_heading_factor = config.heading_factor;
}

} /* namespace rrg_nbv_exploration */
