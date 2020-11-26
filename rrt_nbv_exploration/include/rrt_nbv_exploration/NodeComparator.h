/*
 * NodeComparator.h
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#ifndef RRT_NBV_EXPLORATION_SRC_NODECOMPARATOR_H_
#define RRT_NBV_EXPLORATION_SRC_NODECOMPARATOR_H_

#include "ros/ros.h"
#include <rrt_nbv_exploration_msgs/Tree.h>
#include <rrt_nbv_exploration_msgs/Node.h>
#include <rrt_nbv_exploration/RneMode.h>
#include <stack>

namespace rrt_nbv_exploration {

/**
 * @brief Structure to store the node and the particular gain-cost-ratio and if applicable the nodes in the horizon
 */
struct CompareStruct {
	int node;
	double gain_cost_ratio;
	double distance_to_robot;

	/**
	 * @brief Constructor to initialize struct with node index in tree list
	 */
	CompareStruct(int n) {
		node = n;
		gain_cost_ratio = 0.0;
		distance_to_robot = -1.0;
	}
};

/**
 * @brief Structure to store the next nodes of a previous node while calculating the horizon gain-cost-ratio depth-first
 */
struct HorizonStruct {
	int previous_node;
	std::stack<int> nodes;
	double gain_cost_ratio;

	/**
	 * @brief Constructor to initialize struct with a list of the next nodes of a previous node and the gain-cost-ratio of that node
	 */
	HorizonStruct(int prev, std::vector n, double gcr) {
		previous_node = prev;
		nodes(n);
		gain_cost_ratio = gcr;
	}
};

/**
 * @brief Class to maintain the list of nodes ordered by gain-cost-ratio
 */
class NodeComparator {
public:
	NodeComparator();
	~NodeComparator();

	/**
	 * @brief Initialize variables and maintained list
	 */
	void initialization();

	/**
	 * @brief Sort list and recalculate gain-cost-ratios if necessary
	 * @param Current tree
	 */
	void maintainList(rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Clear the maintained list of ordered nodes
	 */
	void clear();

	/**
	 * @brief Add a new node to the maintained list of ordered nodes
	 * @param Index of the node in the tree
	 */
	void addNode(int node);

	/**
	 * @brief Remove the given node from the maintained list of ordered nodes
	 * @param Index of the node in the tree
	 */
	void removeNode(int node);

	/**
	 * @brief Get the node with the best cost-gain-ratio
	 * @return Index of the node with the best cost-gain-ratio in the tree
	 */
	int getBestNode();

	/**
	 * @brief Check if the maintained list of ordered nodes is empty
	 * @return If the list is empty or not
	 */
	bool isEmpty();

	/**
	 * @brief Triggers a recalculation of all node's path distances and a sorting of the list
	 */
	void robotMoved();

private:
	/**
	 * @brief All nodes (their position in the rrt node list) and their respective gain-cost-ratio ordered ascendingly
	 */
	std::list<CompareStruct> _nodes_ordered_by_gcr;
	/**
	 * @brief All nodes (their position in the rrt node list) and their respective horizon gain-cost-ratio ordered ascendingly
	 */
	std::list<CompareStruct> _nodes_ordered_by_hgcr;
	/**
	 * @brief Operating mode of RNE (classic=0, horizon=1, receding_horizon=2)
	 */
	RneMode _rne_mode;
	/**
	 * @brief Should the list be sorted or not
	 */
	bool _sort_list;
	/**
	 * @brief Did the robot move or not? Implies that all gain-cost-ratios must be recalculated
	 */
	bool _robot_moved;
	/**
	 * @brief Fixed length of the tree's edges, flexible if set to -1 (TODO: wavefront if set to 0)
	 */
	double _edge_length;
	/**
	 * @brief Number of nodes to be considered for horizon evaluation, infinite if set to <= 0
	 */
	int _horizon_length;

	/**
	 * @brief Sorts list of nodes with a gain function which depends on the RNE mode, the node with the best
	 * gain-cost-ratio comes first
	 * @param Current tree
	 */
	void sortByGain(rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Calculate the distances of all paths depending on the edge length (if lesser equals 0, retrieve
	 * particular distances)
	 * @param Current tree
	 */
	void calculatePathDistances(rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Calculate the distance of the given path depending on the edge length (if lesser equals 0, retrieve
	 * particular distances)
	 * @param Current tree
	 * @param Path to calculate distance from
	 */
	double calculatePathDistance(rrt_nbv_exploration_msgs::Tree &rrt,
			std::vector<int> path);

	/**
	 * @brief Calculates the gain-cost-ratio of each node in the list of nodes
	 * @param Current tree
	 */
	void calculateGainCostRatios(rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Calculates the horizon gain-cost-ratio of each node in the list of nodes
	 * @param Current tree
	 */
	void calculateHorizonGainCostRatios(rrt_nbv_exploration_msgs::Tree &rrt);

	/**
	 * @brief Calculates the horizon gain-cost-ratio of the given node
	 * @param Current tree
	 * @param Index of the node which gain-cost-ratio should be calculated
	 * @return The compare struct for the given node with calculated gain-cost-ratio
	 */
	CompareStruct calculateHorizonGainCostRatio(
			rrt_nbv_exploration_msgs::Tree &rrt, int node);

	/**
	 * @brief Returns the gain-cost-ratio of the given node index added to the previous layer horizon gain-cost-ratio
	 * @param Reference to horizon list
	 * @param Node index of which the gain-cost-ratio should be returned
	 * @return Horizon gain-cost-ratio for given node
	 */
	double getCurrentHorizonGainCostRatio(std::stack<HorizonStruct> &horizon_list, int node);

	/**
	 * @brief Compares the two given nodes and returns true if the first node's gain-cost-ratio is better than the second
	 * @param First node
	 * @param Second node
	 * @param Returns if the first node's gain-cost-ratio is better than the second node's gain-cost-ratio
	 */
	bool compareNodeByRatios(const CompareStruct &node_one,
			const CompareStruct &node_two);

};

} /* namespace rrt_nbv_exploration */

#endif /* RRT_NBV_EXPLORATION_SRC_NODECOMPARATOR_H_ */
