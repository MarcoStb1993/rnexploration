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

	_recluster = false;
}

int FrontierClusterer::getBestFrontierNode() {
	return _frontiers.front().center_node_index;
}

void FrontierClusterer::maintainFrontiers(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	if (_recluster) {
		clusterGainClusters(rrg);
		sortFrontiers();
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
	_frontiers.clear();
	int counter = 0;
	for (auto &gain_cluster : rrg.gain_cluster) { //set all gain clusters to unvisited
		gain_cluster.frontier = -1;
	}
	_gain_cluster_searcher->rebuildIndex(rrg);
	//DBSCAN
	for (auto &gain_cluster : rrg.gain_cluster) {
		if (gain_cluster.frontier != -1)  //unvisited gain cluster
			continue;
		gain_cluster.frontier = ++counter;
		_frontiers.emplace_back(counter, gain_cluster,
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
				_frontiers.back().addPoint(
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
}

void FrontierClusterer::sortFrontiers() {
	_frontiers.sort(
			[this](FrontierCluster cluster_one, FrontierCluster cluster_two) {
				return compareFrontiers(cluster_one, cluster_two);
			});
}

bool FrontierClusterer::compareFrontiers(const FrontierCluster &cluster_one,
		const FrontierCluster &cluster_two) {
	return cluster_one.highest_gcr >= cluster_two.highest_gcr;
}

double FrontierClusterer::calculateGainCostRatio(
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::GainCluster &gain_cluster) {
	return rrg.nodes[gain_cluster.node_index].gain
			* exp(-1 * rrg.nodes[gain_cluster.node_index].distanceToRobot);
}

} /* namespace rrg_nbv_exploration */
