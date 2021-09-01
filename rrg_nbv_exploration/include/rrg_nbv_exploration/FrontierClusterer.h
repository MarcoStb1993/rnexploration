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
 * @brieft Structure to store important references to a gain cluster and the node it is at as well as
 * the gain clusters GCR
 */
struct GainClusterInfo {
	int gain_cluster_index, node_index;
	double gcr;

	GainClusterInfo(int n_index, int gc_index, double gain_cost_ratio) {
		ROS_INFO_STREAM(
				"init gain cluster info with " << gc_index << " at node " << n_index << " with gcr " << gain_cost_ratio);
		node_index = n_index;
		gain_cluster_index = gc_index;
		gcr = gain_cost_ratio;
	}

	void resetGcr(double gain_cost_ratio) {
		gcr = gain_cost_ratio;
	}
};

/**
 * Structure to store frontier clusters which consists of one or multiple gain clusters in proximity
 * to each other. Its center position, size and gain (average and summed total) will be used to define
 * the next goal.
 */
struct FrontierCluster {
	int number;
	int size, center_node_index, highest_gcr_index;
	std::vector<GainClusterInfo> gain_clusters;
	std::vector<int> frontier_edges;

	/**
	 * @brief Constructor for robot position as frontier
	 */
	FrontierCluster(int num, geometry_msgs::Point pos, int index) {
		number = num;
		center_node_index = index;
		highest_gcr_index = -1;
		size = 0;
	}
	/**
	 * @brief Constructor to initialize struct from gain cluster and GCR
	 */
	FrontierCluster(int num,
			rrg_nbv_exploration_msgs::GainCluster &gain_cluster, double gcr) {
		ROS_INFO_STREAM(
				"init frontier cluster "<<num<< " with gain cluster " << gain_cluster.index << " at node " << gain_cluster.node_index << " with gcr " << gcr);
		number = num;
		center_node_index = gain_cluster.node_index;
		size = 1;
		gain_clusters.emplace_back(gain_cluster.node_index, gain_cluster.index,
				gcr);
		highest_gcr_index = 0;
	}

	void addPoint(rrg_nbv_exploration_msgs::GainCluster &gain_cluster,
			double gcr) {
		ROS_INFO_STREAM(
				"frontier " << number << ": add point " << gain_cluster.index << " at node " << gain_cluster.node_index << " with gcr " << gcr);
		gain_clusters.emplace_back(gain_cluster.node_index, gain_cluster.index,
				gcr);
		size++;
		ROS_INFO_STREAM(
				"added point, size " << size << " highest gcr index " << highest_gcr_index);
		if (highest_gcr_index == -1
				|| gcr > gain_clusters[highest_gcr_index].gcr) {
			ROS_INFO_STREAM("new highest gcr");
			center_node_index = gain_cluster.node_index;
			highest_gcr_index = gain_clusters.size() - 1;
		}
		ROS_INFO_STREAM("finito");
	}

	void removePoint(rrg_nbv_exploration_msgs::GainCluster &gain_cluster) {
		bool removed_highest_gcr = gain_cluster.index
				== gain_clusters[highest_gcr_index].gain_cluster_index;
		gain_clusters.erase(gain_clusters.begin() + highest_gcr_index);
		size--;
		if (removed_highest_gcr) {
			double highest_gcr = 0;
			for (auto &cluster : gain_clusters) {
				if (cluster.gcr > highest_gcr) {
					highest_gcr = cluster.gcr;
					highest_gcr_index = cluster.gain_cluster_index;
				}
			}
			center_node_index = gain_clusters[highest_gcr_index].node_index;
		}
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
	 * @brief Recalculates the frontier clusters if necessary (robot moved or gain clusters changed)
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

	void gainClusterRemoved();
	/**
	 * @brief Adds a node with its gain clusters to be clustered into a frontier
	 * @param RRG
	 * @param Index of node to be added
	 */
	void addNode(rrg_nbv_exploration_msgs::Graph &rrg, int node);

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
	 * @brief Global frontier counter
	 */
	int _frontier_counter;

	/**
	 * @brief Executes DBSCAN to cluster gain clusters into frontier clusters
	 * @param List of gain clusters which server as starting set for clustering
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

	std::vector<rrg_nbv_exploration_msgs::GainCluster> getNodesGainCluster(
			rrg_nbv_exploration_msgs::Graph &rrg, int node);
	void visualizeTspRoute(rrg_nbv_exploration_msgs::Graph &rrg);
	void handleNewCorePoint(rrg_nbv_exploration_msgs::Graph &rrg,
			int core_point_index);
	void changeGainClustersFrontier(rrg_nbv_exploration_msgs::Graph &rrg,
			const std::vector<int> &affected_gain_clusters, int frontier_index);
};

} /* namespace rrg_nbv_exploration */

#endif /* RRG_NBV_EXPLORATION_SRC_FRONTIERCLUSTERER_H_ */
