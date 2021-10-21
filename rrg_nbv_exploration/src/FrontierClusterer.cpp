/*
 * FrontierClusterer.cpp
 *
 *  Created on: Aug 9, 2021
 *      Author: marco
 */

#include <rrg_nbv_exploration/FrontierClusterer.h>

namespace rrg_nbv_exploration {

FrontierClusterer::FrontierClusterer() {
}

FrontierClusterer::~FrontierClusterer() {
}

void FrontierClusterer::initialize(rrg_nbv_exploration_msgs::Graph &rrg) {
	ros::NodeHandle private_nh("~");
	double eps_distance;
	private_nh.param("eps_distance", eps_distance, 2.0);
	_cluster_distance_squared = pow(eps_distance, 2);
	private_nh.param("min_neighbors", _min_neighbors, 2);
	_gain_cluster_searcher.reset(new GainClusterSearcher());
	_gain_cluster_searcher->initialize(rrg);

	_tsp_route_pub = _nh.advertise<nav_msgs::Path>("tsp_route", 1, false);

	_recluster = false;
}

int FrontierClusterer::getBestFrontierNode() {
	return _frontiers.front().center_node_index;
}

void FrontierClusterer::maintainFrontiers(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	if (_recluster) {
		clusterGainClusters(rrg);
		if (_frontiers.size() > 1)
			orderFrontiersWithTsp(rrg);
		else
			_frontiers.clear(); //only robot position as frontier present, remove
//		ROS_INFO_STREAM("Sorted Frontiers");
//		for (auto frontier : _frontiers) {
//			ROS_INFO_STREAM(
//					"Frontier " << frontier.number << " at node " << frontier.center_node_index);
//		}
		_recluster = false;
	}
	if (_tsp_route_pub.getNumSubscribers() > 0) {
		nav_msgs::Path tsp_route;
		ros::Time timestamp = ros::Time::now();
		tsp_route.header.frame_id = "map";
		tsp_route.header.stamp = timestamp;
		for (auto frontier : _frontiers) {
			geometry_msgs::PoseStamped path_pose;
			path_pose.header.frame_id = "map";
			path_pose.header.stamp = timestamp;
			path_pose.pose.position =
					rrg.nodes[frontier.center_node_index].position;
			path_pose.pose.position.z += 0.1; //elevate route above RRG
			double yaw = M_PI * rrg.nodes[frontier.center_node_index].best_yaw
					/ 180.0;
			tf2::Quaternion quaternion;
			quaternion.setRPY(0, 0, yaw);
			quaternion.normalize();
			path_pose.pose.orientation = tf2::toMsg(quaternion);
			tsp_route.poses.push_back(path_pose);
		}
		_tsp_route_pub.publish(tsp_route);
	}
}

bool FrontierClusterer::isEmpty() {
	return _frontiers.empty();
}

void FrontierClusterer::robotMoved() {
	_recluster = true;
}

void FrontierClusterer::gainClusterChanged() {
	_recluster = true;
}

void FrontierClusterer::clusterGainClusters(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<FrontierCluster> new_frontiers;
	std::vector<FrontierChange> frontier_changes; //pair of previous frontier (first) and new frontier (second)

	int counter = 0;
	new_frontiers.emplace_back(counter, rrg.nodes[rrg.nearest_node].position,
			rrg.nearest_node); //robot position as start for TSP
	for (auto &gain_cluster : rrg.gain_cluster) { //set all gain clusters to unvisited
		gain_cluster.previous_frontier = gain_cluster.frontier;
		gain_cluster.frontier = -1;
	}
	_gain_cluster_searcher->rebuildIndex(rrg);
	//DBSCAN
	for (auto &gain_cluster : rrg.gain_cluster) {
		if (gain_cluster.frontier != -1)  //unvisited gain cluster
			continue;
		gain_cluster.frontier = ++counter;
		if (gain_cluster.previous_frontier != -1)
			addFrontierChange(frontier_changes, gain_cluster.previous_frontier,
					gain_cluster.frontier);
		new_frontiers.emplace_back(counter, gain_cluster,
				calculateGainCostRatio(rrg, gain_cluster));
		std::queue<int> gain_cluster_queue;
		std::vector<int> neighbor_gain_clusters =
				_gain_cluster_searcher->searchInRadius(gain_cluster.position,
						_cluster_distance_squared);
		if (neighbor_gain_clusters.size() >= _min_neighbors) {
			for (auto &neighbor_gain_cluster : neighbor_gain_clusters) {
				gain_cluster_queue.push(neighbor_gain_cluster);
			}
		}
		while (!gain_cluster_queue.empty()) {
			if (rrg.gain_cluster[gain_cluster_queue.front()].frontier == -1) {
				rrg.gain_cluster[gain_cluster_queue.front()].frontier = counter;
				if (gain_cluster.previous_frontier != -1)
					addFrontierChange(frontier_changes,
							gain_cluster.previous_frontier,
							gain_cluster.frontier);
				new_frontiers.back().addPoint(
						rrg.gain_cluster[gain_cluster_queue.front()],
						calculateGainCostRatio(rrg,
								rrg.gain_cluster[gain_cluster_queue.front()]));
				neighbor_gain_clusters = _gain_cluster_searcher->searchInRadius(
						rrg.gain_cluster[gain_cluster_queue.front()].position,
						_cluster_distance_squared);
				if (neighbor_gain_clusters.size() >= _min_neighbors) {
					for (auto &neighbor_gain_cluster : neighbor_gain_clusters) {
						gain_cluster_queue.push(neighbor_gain_cluster);
					}
				}
			}
			gain_cluster_queue.pop();
		}
	}
	mapNewFrontiersToExisting(new_frontiers, frontier_changes);
}

void FrontierClusterer::addFrontierChange(
		std::vector<FrontierChange> &frontier_changes, int previous_frontier,
		int new_frontier) {
	ROS_INFO_STREAM(
			"Add frontier change from " << previous_frontier << " to " << new_frontier);
	int changes_detected = -1;
	if (frontier_changes.size() > 0) {
		auto it = frontier_changes.begin();
		while (it != frontier_changes.end()) {
			if (it->checkForFrontier(previous_frontier, new_frontier)) {
				if (changes_detected >= 0) {
					ROS_INFO_STREAM(
							"Merge changes from " << it-frontier_changes.begin()<< " into " << changes_detected);
					frontier_changes[changes_detected].addFrontierChanges(
							it->previous_frontiers, it->new_frontiers);
					frontier_changes.erase(it);
				} else {
					it->addFrontierChange(previous_frontier, new_frontier);
					changes_detected = it - frontier_changes.begin();
					ROS_INFO_STREAM("Add new change to " << changes_detected);
				}
			}
			it++;
		}

	}
	if (changes_detected == -1) {
		frontier_changes.emplace_back(previous_frontier, new_frontier);
		ROS_INFO_STREAM("Add new change " << frontier_changes.size()-1);
	}

	for (auto change : frontier_changes) {
		std::string from, to;
		for (auto index : change.previous_frontiers) {
			from += std::to_string(index) + ", ";
		}
		for (auto index : change.new_frontiers) {
			to += std::to_string(index) + ", ";
		}
		ROS_INFO_STREAM("Frontier change from: " << from << " to: " << to);
	}
}

void FrontierClusterer::mapNewFrontiersToExisting(
		const std::vector<FrontierCluster> &new_frontiers,
		std::vector<FrontierChange> frontier_changes) {
	//iterate over previous frontiers and replace frontiers with new frontiers according to changes:
	//check for each previous frontier if it exists in a change and replace it with new frontier(s),
	//delete other previous frontiers from list if necessary
	//check for every replacement if center node remained the same, if not recalculate frontier edges
	//for it and rerun TSP solver, if none changed, no TSP solver

	for (auto change : frontier_changes) {

		std::vector<FrontierCluster>::iterator it;
		int index = 0;
		do {
			it = std::find_if(new_frontiers.begin(), new_frontiers.end(),
					[change, index](FrontierCluster frontier) {
						return frontier.number
								== change.previous_frontiers[index];
					});
			index++;
		} while (it != new_frontiers.end());

	}
//only recalculate frontier edges for frontiers which center changed
	_frontiers = new_frontiers;
}

double FrontierClusterer::calculateGainCostRatio(
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::GainCluster &gain_cluster) {
	return rrg.nodes[gain_cluster.node_index].gain
			* exp(-1 * rrg.nodes[gain_cluster.node_index].distanceToRobot);
}

void FrontierClusterer::orderFrontiersWithTsp(
		rrg_nbv_exploration_msgs::Graph &rrg) {
//	ROS_INFO_STREAM("order with TSP");
	std::vector<int> route;
	for (auto &frontier : _frontiers) {
		route.push_back(frontier.number);
	}
	route.push_back(0); //robot position as end, replace with root for homing
	calculateFrontierGraphEdges(rrg, route);
//	std::string route_info = "Route: ";
//	for (auto n : route) {
//		route_info += std::to_string(n) + " ("
//				+ std::to_string(_frontiers[n].center_node_index) + ")" + "->";
//	}
//	route_info += " with distance=" + std::to_string(calculateDistance(route));
//	ROS_INFO_STREAM(route_info);
//	ROS_INFO_STREAM("Calculated edges");
	bool improved = true;
	while (improved) {
		improved = false;
		double best_distance = calculateDistance(route);
//		ROS_INFO_STREAM("Start distance: " << best_distance);
		for (int i = 1; i < route.size() - 1; i++) { //omit first "frontier" (robot position)
			for (int k = i + 1; k < route.size() - 1; k++) { //omit last "frontier" (robot position)
				std::vector<int> new_route = twoOptSwap(route, i, k);
				double new_distance = calculateDistance(new_route);
//				ROS_INFO_STREAM(
//						"Swapped " << i << " to " << k << " new distance: " << new_distance);
				if (new_distance < best_distance) {
					route = new_route;
					best_distance = new_distance;
					improved = true;
				}
			}
		}
//		route_info = "Route: ";
//		for (auto n : route) {
//			route_info += std::to_string(n) + " ("
//					+ std::to_string(_frontiers[n].center_node_index) + ")"
//					+ "->";
//		}
//		route_info += " with distance=" + std::to_string(best_distance);
//		ROS_INFO_STREAM(route_info);
	}
//	ROS_INFO_STREAM("Finished 2-opt");
	std::vector<FrontierCluster> frontiers;
	for (int m = 1; m < route.size() - 1; m++) {
		frontiers.push_back(_frontiers[route[m]]);
	}
	_frontiers = frontiers;
//	ROS_INFO_STREAM("Sorted frontiers");
}

void FrontierClusterer::calculateFrontierGraphEdges(
		rrg_nbv_exploration_msgs::Graph &rrg, std::vector<int> &route) {
//	ROS_INFO_STREAM("Calculate graph edges");
	for (int i = 0; i < route.size() - 2; i++) { //omit last two loops because of symmetrical graph edges (1) and start node again (2)
		calculateFrontierGraphEdge(rrg, route, route[i]);
	}
}

void FrontierClusterer::calculateFrontierGraphEdge(
		rrg_nbv_exploration_msgs::Graph &rrg, std::vector<int> &route,
		int frontier) {
	int node_index = _frontiers[frontier].center_node_index;
//	ROS_INFO_STREAM(
//			"Calculate graph edge for frontier " << frontier << " at node " << node_index);
	std::vector<double> distances(rrg.node_counter,
			std::numeric_limits<double>::infinity()); //list of distances to all nodes
	distances[node_index] = 0.0;
	std::set<std::pair<double, int>> node_queue;
	node_queue.insert(std::make_pair(distances[node_index], node_index));
	while (!node_queue.empty()) { //Dijkstra's algorithm
		double current_distance = node_queue.begin()->first;
		int current_node = node_queue.begin()->second;
		node_queue.erase(node_queue.begin());
//		ROS_INFO_STREAM(
//				"Looking at node " << current_node << " with distance to frontier " << current_distance);

		for (auto edge : rrg.nodes[current_node].edges) {
			int neighbor_node =
					rrg.edges[edge].first_node == current_node ?
							rrg.edges[edge].second_node :
							rrg.edges[edge].first_node;
			double neighbor_distance = rrg.edges[edge].length;
			double total_distance = current_distance + neighbor_distance;
//			ROS_INFO_STREAM(
//					"Neighbor " << neighbor_node << " with edge length " << neighbor_distance << " has total distance " << total_distance << " compared to current: " << distances[neighbor_node] << (total_distance < distances[neighbor_node] ? " shorter": " longer"));
			if (total_distance < distances[neighbor_node]) {
				node_queue.erase(
						std::make_pair(distances[neighbor_node],
								neighbor_node));
				distances[neighbor_node] = total_distance;
				node_queue.insert(
						std::make_pair(distances[neighbor_node],
								neighbor_node));
//				ROS_INFO_STREAM("set node " << neighbor_node << "'s path");
			}
		}
	}
//	ROS_INFO_STREAM("Dijkstra finished");
	for (auto index : route) { //save all distances to other frontiers
		if (index != frontier
				&& !frontierEdgePresent(std::min(frontier, index),
						std::max(frontier, index))) {
			_frontier_edges.emplace_back(std::min(frontier, index),
					std::max(frontier, index),
					distances[_frontiers[index].center_node_index]);
			_frontiers[index].addFrontierEdge(_frontier_edges.size() - 1);
			_frontiers[frontier].addFrontierEdge(_frontier_edges.size() - 1);
//			ROS_INFO_STREAM(
//					"Add frontier edge " << (frontier < index ? frontier : index) << "->"<< (frontier < index ? index : frontier) << " with distance: " << distances[_frontiers[index].center_node_index]);
		}
	}
}

bool FrontierClusterer::frontierEdgePresent(int first_node, int second_node) {
	bool present = false;
	for (auto &edge : _frontier_edges) {
		if (edge.first_node_index == first_node
				&& edge.second_node_index == second_node)
			present = true;
	}
	return present;
}

double FrontierClusterer::calculateDistance(std::vector<int> &route) {
	double distance = 0.0;
	for (int i = 1; i < route.size(); i++) {
//		ROS_INFO_STREAM("To Frontier node " << route[i]);
		for (auto edge_index : _frontiers[route[i]].frontier_edges) {
			if (_frontier_edges[edge_index].first_node_index
					== std::min(route[i - 1], route[i])
					&& _frontier_edges[edge_index].second_node_index
							== std::max(route[i - 1], route[i])) {
//				ROS_INFO_STREAM(
//						"Frontier edge " << _frontier_edges[edge_index].first_node_index << "->" << _frontier_edges[edge_index].second_node_index << " distance=" << _frontier_edges[edge_index].distance);
				distance += _frontier_edges[edge_index].distance;
				break;
			}
		}
	}
	return distance;
}

std::vector<int> FrontierClusterer::twoOptSwap(std::vector<int> &route, int i,
		int k) {
	std::vector<int> new_route;
	for (int j = 0; j < i; j++) {
		new_route.push_back(route[j]);
	}
	for (int l = k; l >= i; l--) {
		new_route.push_back(route[l]);
	}
	for (int m = k + 1; m < route.size(); m++) {
		new_route.push_back(route[m]);
	}
	return new_route;
}

} /* namespace rrg_nbv_exploration */
