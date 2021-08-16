/*
 * FrontierClusterer.h
 *
 *  Created on: Aug 9, 2021
 *      Author: marco
 */

#ifndef RRG_NBV_EXPLORATION_SRC_FRONTIERCLUSTERER_H_
#define RRG_NBV_EXPLORATION_SRC_FRONTIERCLUSTERER_H_

#include "ros/ros.h"
#include <rrg_nbv_exploration_msgs/Graph.h>
#include <rrg_nbv_exploration_msgs/Node.h>
#include <rrg_nbv_exploration_msgs/GainCluster.h>
#include <rrg_nbv_exploration/GainClusterSearcher.h>
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <queue>

/**
 * Structure to store frontier clusters which consists of one or multiple gain clusters in proximity
 * to each other. Its center position, size and gain (average and summed total) will be used to define
 * the next goal.
 */
struct FrontierCluster {
	int number;
	geometry_msgs::Point center;
	int size, total_gain, center_node_index;
	double highest_gcr;
	std::vector<std::pair<int, int>> gain_clusters; //pairs of node index (first) and gain cluster index (second)

	/**
	 * @brief Constructor to initialize struct from gain cluster and GCR
	 */
	FrontierCluster(int num,
			rrg_nbv_exploration_msgs::GainCluster &gain_cluster, double gcr) {
		number = num;
		center = gain_cluster.position;
		center_node_index = gain_cluster.node_index;
		size = 1;
		total_gain = gain_cluster.size;
		highest_gcr = gcr;
		gain_clusters.push_back(
				std::make_pair(gain_cluster.node_index, gain_cluster.index));
	}

	void addPoint(rrg_nbv_exploration_msgs::GainCluster &gain_cluster,
			double gcr) {
		total_gain += gain_cluster.size;
		if (gcr > highest_gcr) {
			center = gain_cluster.position;
			center_node_index = gain_cluster.node_index;
			highest_gcr = gcr;
		}
		gain_clusters.push_back(
				std::make_pair(gain_cluster.node_index, gain_cluster.index));
	}
};

namespace rrg_nbv_exploration {

/**
 * @brief The frontier clusterer identifies frontiers by clustering the gain clusters of nodes
 */
class FrontierClusterer {
public:
	FrontierClusterer();
	virtual ~FrontierClusterer();
	/**
	 * @brief Initialize parameters and nearest neighbor search
	 */
	void initialize(rrg_nbv_exploration_msgs::Graph &rrg);
	/**
	 * @brief Retrieves the frontier center with the best GCR
	 * @return Node index with the best GCR
	 */
	int getBestFrontierNode();
	/**
	 * @brief Recalculates the frontier clusters if necesary (robot moved or gain clusters changed)
	 * @param RRG
	 */
	void maintainFrontiers(rrg_nbv_exploration_msgs::Graph &rrg);
	/**
	 * @brief Check if the list of frontiers is empty
	 * @return If the list is empty or not
	 */
	bool isEmpty();
	/**
	 * @brief Triggers a recalculation of all frontier clusters
	 */
	void robotMoved();
	/**
	 * @brief Triggers a recalculation of all frontier clusters
	 */
	void gainClusterChanged();

private:
	/**
	 * @brief Helper class for kd-tree FrontierCluster and nearest neighbor search
	 */
	std::shared_ptr<GainClusterSearcher> _gain_cluster_searcher;

	/**
	 * @brief List of frontier clusters
	 */
	std::list<FrontierCluster> _frontiers;
	/**
	 * @brief Squared distance for gain clusters to count to a frontier cluster (same as max edge length)
	 */
	double _cluster_distance_squared;
	/**
	 * @brief Minimum number of neighbors to count a gain cluster as a core point for a frontier cluster
	 */
	int _min_neighbors;
	/**
	 * @brief If the frontier clusters need to be recalculated
	 */
	bool _recluster;

	/**
	 * @brief Executes DBSCAN to cluster gain clusters into frontier clusters
	 * @param RRG
	 */
	void clusterGainClusters(rrg_nbv_exploration_msgs::Graph &rrg);
	/**
	 * @brief Sort frontiers by their best GCRs
	 */
	void sortFrontiers();
	/**
	 * @brief Compare a frontier's GCR to another frontier's GCR
	 * @param First frontier
	 * @param Second frontier
	 */
	bool compareFrontiers(const FrontierCluster &cluster_one,
			const FrontierCluster &cluster_two);
	/**
	 * @brief Calculate the GCR for a gain cluster
	 * @param RRG
	 * @param Gain cluster
	 */
	double calculateGainCostRatio(rrg_nbv_exploration_msgs::Graph &rrg,
			rrg_nbv_exploration_msgs::GainCluster &gain_cluster);

};

} /* namespace rrg_nbv_exploration */

#endif /* RRG_NBV_EXPLORATION_SRC_FRONTIERCLUSTERER_H_ */
