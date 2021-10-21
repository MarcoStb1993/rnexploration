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
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
	std::vector<int> frontier_edges;

	/**
	 * @brief Constructor for robot position as frontier
	 */
	FrontierCluster(int num, geometry_msgs::Point pos, int index) {
		number = num;
		center = pos;
		center_node_index = index;
		size = 0;
		total_gain = 0;
		highest_gcr = 0;
	}
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

	void addFrontierEdge(int index) {
		frontier_edges.push_back(index);
	}
};

/**
 * @brief Structure to store edges between frontier nodes consisting of a summed distance of the
 * path in the RRG
 */
struct FrontierEdge {
	int first_node_index, second_node_index;
	double distance;

	FrontierEdge(int first, int second, double dist) {
		first_node_index = first;
		second_node_index = second;
		distance = dist;
	}
};

/**
 * @brief Structure to store changes between old and new frontiers in two lists with the previous
 * frontier index and the new frontier index
 */
struct FrontierChange {
	std::vector<int> previous_frontiers, new_frontiers;

	FrontierChange(int previous_frontier, int new_frontier) {
		previous_frontiers.push_back(previous_frontier);
		new_frontiers.push_back(new_frontier);
	}

	bool checkForFrontier(int previous_frontier, int new_frontier) {
		if (std::find(previous_frontiers.begin(), previous_frontiers.end(),
				previous_frontier) != previous_frontiers.end()
				|| std::find(new_frontiers.begin(), new_frontiers.end(),
						new_frontier) != new_frontiers.end())
			return true;
		else
			return false;
	}

	void addFrontierChange(int previous_frontier, int new_frontier) {
		auto previous_result = std::find(previous_frontiers.begin(),
				previous_frontiers.end(), previous_frontier);
		if (previous_result == previous_frontiers.end())
			previous_frontiers.push_back(previous_frontier);
		auto new_result = std::find(new_frontiers.begin(), new_frontiers.end(),
				new_frontier);
		if (new_result == new_frontiers.end())
			new_frontiers.push_back(new_frontier);
	}

	void addFrontierChanges(std::vector<int> prev_frontiers,
			std::vector<int> n_frontiers) {
		for (auto prev_frontier : prev_frontiers) {
			auto previous_result = std::find(previous_frontiers.begin(),
					previous_frontiers.end(), prev_frontier);
			if (previous_result == previous_frontiers.end())
				previous_frontiers.push_back(prev_frontier);
		}
		for (auto n_frontier : n_frontiers) {
			auto new_result = std::find(new_frontiers.begin(),
					new_frontiers.end(), n_frontier);
			if (new_result == new_frontiers.end())
				new_frontiers.push_back(n_frontier);
		}
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
	 * @brief Check if the list of frontiers is empty (only current position present)
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
	ros::NodeHandle _nh;
	ros::Publisher _tsp_route_pub;

	/**
	 * @brief Helper class for kd-tree FrontierCluster and nearest neighbor search
	 */
	std::shared_ptr<GainClusterSearcher> _gain_cluster_searcher;

	/**
	 * @brief List of frontier clusters
	 */
	std::vector<FrontierCluster> _frontiers;
	/**
	 * @brief List of edges connecting the frontiers
	 */
	std::vector<FrontierEdge> _frontier_edges;
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
	 * @brief Calculate the GCR for a gain cluster
	 * @param RRG
	 * @param Gain cluster
	 */
	double calculateGainCostRatio(rrg_nbv_exploration_msgs::Graph &rrg,
			rrg_nbv_exploration_msgs::GainCluster &gain_cluster);

	void addFrontierChange(std::vector<FrontierChange> &frontier_changes,
			int previous_frontier, int new_frontier);

	void mapNewFrontiersToExisting(
			const std::vector<FrontierCluster> &new_frontiers,
			std::vector<FrontierChange> frontier_changes);

	/**
	 * @brief Find the shortest route through all frontiers using 2-opt heuristic for TSP and order
	 * them according to this route with the robot position as start and end point
	 * @param RRG
	 */
	void orderFrontiersWithTsp(rrg_nbv_exploration_msgs::Graph &rrg);

	void calculateFrontierGraphEdges(rrg_nbv_exploration_msgs::Graph &rrg,
			std::vector<int> &route);

	void calculateFrontierGraphEdge(rrg_nbv_exploration_msgs::Graph &rrg,
			std::vector<int> &route, int frontier);

	bool frontierEdgePresent(int first_node, int second_node);

	double calculateDistance(std::vector<int> &route);

	std::vector<int> twoOptSwap(std::vector<int> &route, int i, int k);
};

} /* namespace rrg_nbv_exploration */

#endif /* RRG_NBV_EXPLORATION_SRC_FRONTIERCLUSTERER_H_ */
