/*
 * NodeComparator.h
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#ifndef rrg_nbv_exploration_SRC_NODECOMPARATOR_H_
#define rrg_nbv_exploration_SRC_NODECOMPARATOR_H_

#include "ros/ros.h"
#include <rrg_nbv_exploration_msgs/Graph.h>
#include <rrg_nbv_exploration_msgs/Node.h>
#include <stack>

#include <fstream>

namespace rrg_nbv_exploration {

/**
 * @brief Structure to store the node and the particular gain-cost-ratio
 */
struct CompareStruct {
	int node;
	double gain_cost_ratio;

	/**
	 * @brief Constructor to initialize struct with node index in graph list
	 */
	CompareStruct(int n) {
		node = n;
		gain_cost_ratio = 0.0;
	}

	/**
	 * @brief Constructor to initialize struct with node index in graph list and gain-cost-ratio
	 */
	CompareStruct(int n, double gcr) {
		node = n;
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
	 * @param Current graph
	 */
	void maintainList(rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Clear the maintained list of ordered nodes
	 */
	void clear();

	/**
	 * @brief Add a new node to the maintained list of ordered nodes
	 * @param Index of the node in the graph
	 */
	void addNode(int node);

	/**
	 * @brief Remove the given node from the maintained list of ordered nodes
	 * @param Index of the node in the graph
	 */
	void removeNode(int node);

	/**
	 * @brief Get the node with the best cost-gain-ratio
	 * @return Index of the node with the best cost-gain-ratio in the tree
	 */
	int getBestNode();

	/**
	 * @brief Get the number of nodes currently in the list
	 * @return The size of the list
	 */
	int getListSize();

	/**
	 * @brief Check if the maintained list of ordered nodes is empty
	 * @return If the list is empty or not
	 */
	bool isEmpty();

	/**
	 * @brief Triggers a recalculation of all node's path distances and a sorting of the list
	 */
	void robotMoved();

	/**
	 * @brief Triggers sorting of the list
	 */
	void setSortList();

private:
	/**
	 * @brief All nodes (their position in the rrg node list) and their respective gain-cost-ratio ordered ascendingly
	 */
	std::list<CompareStruct> _nodes_ordered_by_gcr;
	/**
	 * @brief Should the list be sorted or not
	 */
	bool _sort_list;
	/**
	 * @brief Did the robot move or not? Implies that all gain-cost-ratios must be recalculated
	 */
	bool _robot_moved;
	/**
	 * @brief Weighting factor for the information gain of a node
	 */
	double _gain_factor;
	/**
	 * @brief Weighting factor for the distance to a node
	 */
	double _distance_factor;
	/**
	 * @brief Weighting factor for the heading change while moving to a node
	 */
	double _heading_factor;
	/**
	 * @brief Weighting factor for the traversability cost along the path to a node
	 */
	double _traversability_factor;

	/**
	 * @brief Sorts list of nodes with a gain function, the node with the best gain-cost-ratio comes first
	 */
	void sortByGain();

	/**
	 * @brief Calculates the gain-cost-ratio of each node in the list of nodes
	 * @param Current graph
	 */
	void calculateGainCostRatios(rrg_nbv_exploration_msgs::Graph &rrh);

	/**
	 * @brief Returns the gain-cost-ratio of the given node index
	 * @param Node index of which the gain-cost-ratio should be returned
	 * @return Gain-cost-ratio for given node
	 */
	double getNodeGainCostRatio(int node);

	/**
	 * @brief Compares the two given nodes and returns true if the first node's gain-cost-ratio is better than the second
	 * @param First node
	 * @param Second node
	 * @param Returns if the first node's gain-cost-ratio is better than the second node's gain-cost-ratio
	 */
	bool compareNodeByRatios(const CompareStruct &node_one,
			const CompareStruct &node_two);

};

} /* namespace rrg_nbv_exploration */

#endif /* rrg_nbv_exploration_SRC_NODECOMPARATOR_H_ */
