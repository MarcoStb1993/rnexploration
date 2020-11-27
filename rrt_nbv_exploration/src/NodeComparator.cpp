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
	_horizon_length = std::max(1, _horizon_length);	//horizon length must be at least 1

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
		std::string gcr = "GCR: ";
		for (auto it : _nodes_ordered_by_gcr) {
			gcr.append(std::to_string(it.node));
			gcr.append("(");
			gcr.append(std::to_string(it.gain_cost_ratio));
			gcr.append("),");
//			ROS_INFO_STREAM(
//					std::setprecision(2) << "Node " << it.node << " with gcr: " << it.gain_cost_ratio << ", distance: " << it.distance_to_robot <<" and status: " << (int) rrt.nodes[it.node].status);
		}
		ROS_INFO_STREAM(gcr);

		std::string hgcr = "HGCR: ";
		for (auto it2 : _nodes_ordered_by_hgcr) {
			hgcr.append(std::to_string(it2.node));
			hgcr.append("(");
			hgcr.append(std::to_string(it2.gain_cost_ratio));
			hgcr.append("),");
			//			ROS_INFO_STREAM(
			//					std::setprecision(2) << "Node " << it.node << " with gcr: " << it.gain_cost_ratio << ", distance: " << it.distance_to_robot <<" and status: " << (int) rrt.nodes[it.node].status);
		}
		ROS_INFO_STREAM(hgcr);
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
		ROS_INFO_STREAM(
				"Best HGCR node: " << _nodes_ordered_by_hgcr.front().node);
		return _nodes_ordered_by_hgcr.front().node;
	} else {
		ROS_INFO_STREAM(
				"Best GCR node: " << _nodes_ordered_by_gcr.front().node);
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
	if (rrt.nearest_node != 0) //root node has no parent
		start_nodes.push_back(rrt.nodes[rrt.nearest_node].parent);
//	ROS_INFO_STREAM(
//			"Nearest node " << rrt.nearest_node << " has " << start_nodes.size() << " neighbors, horizon length: " << _horizon_length);
	double nearest_node_gcr = getNodeGainCostRatio(rrt.nearest_node);
	if (nearest_node_gcr > 0) {	//add nearest node to list if it still has gain
		ROS_INFO_STREAM(
				std::fixed << std::setprecision(3) << "add nearest node " << rrt.nearest_node << " with gcr " << nearest_node_gcr);
		_nodes_ordered_by_hgcr.emplace_back(rrt.nearest_node, nearest_node_gcr);
	}
	for (auto &node : start_nodes) {
		_nodes_ordered_by_hgcr.push_back(
				calculateHorizonGainCostRatio(rrt, node));
	}
}

CompareStruct NodeComparator::calculateHorizonGainCostRatio(
		rrt_nbv_exploration_msgs::Tree &rrt, int node) {
	ROS_INFO_STREAM("Start node: " << node);
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
//		ROS_INFO_STREAM(
//				std::fixed << std::setprecision(3) << "In node " << current_node << " with last node " << last_node << " and current gcr: " << current_hgcr);
		bool next_node = false;
		if (horizon_list.size() < _horizon_length) { //try to add new nodes if horizon is not reached
			std::vector<int> node_list;
			if (current_node != 0
					&& last_node != rrt.nodes[current_node].parent) { //root node has no parent
				node_list.push_back(rrt.nodes[current_node].parent);
//				ROS_INFO_STREAM("add node: " << rrt.nodes[current_node].parent);
			}
			for (auto child : rrt.nodes[current_node].children) {
				if (last_node != child) {
					node_list.push_back(child);
//					ROS_INFO_STREAM("add node: " << child);
				}
			}
			if (node_list.empty()) { //last node in branch, go to next node
				best_hgcr = std::max(current_hgcr, best_hgcr);
//				ROS_INFO_STREAM(
//						std::fixed << std::setprecision(3) <<"No new nodes, best gcr: " << best_hgcr);
				next_node = true;
			} else { //additional layers to add
				horizon_list.emplace(current_node, node_list, current_hgcr);
				last_node = current_node;
				current_node = horizon_list.top().nodes.top();
				current_horizon.push_back(current_node);
			}
		} else { //horizon reached, go to next node
			best_hgcr = std::max(current_hgcr, best_hgcr);
//			ROS_INFO_STREAM(
//					std::fixed << std::setprecision(3) <<"Horizon reached, best gcr: " << best_hgcr);
			next_node = true;
		}
		if (best_hgcr == current_hgcr)
			best_horizon = current_horizon;
		std::string hor = "Current horizon: ";
		for (auto i : current_horizon) {
			hor.append("(");
			hor.append(std::to_string(i));
			hor.append(",");
			hor.append(std::to_string((int) rrt.nodes[i].status));
			hor.append(")");
		}
		ROS_INFO_STREAM(hor);
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
		//debugging only
		auto *end = &horizon_list.top() + 1;
		auto *begin = end - horizon_list.size();
		std::vector<HorizonStruct> test(begin, end);
		std::string out;
		for (auto i : test) {
			auto *tend = &i.nodes.top() + 1;
			auto *tbegin = tend - i.nodes.size();
			std::vector<int> ttest(tbegin, tend);
			std::string tmp = " {";
			for (auto j : ttest) {
				tmp.append(std::to_string(j));
				tmp.append(" ");
			}
			tmp.append("p:");
			tmp.append(std::to_string(i.previous_node));
			tmp.append(" gcr:");
			tmp.append(std::to_string(i.gain_cost_ratio));
			tmp.append("}");
			out.append(tmp);
		}
		ROS_INFO_STREAM("Stack: " << out);
	} while (!horizon_list.empty());
	for (auto i : best_horizon) { //find first unexplored node in horizon
		if (rrt.nodes[i].status != rrt_nbv_exploration_msgs::Node::EXPLORED) {
			if (i != node) {
				ROS_INFO_STREAM(
						"Change horizon node from " << node << " to " << i);
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

