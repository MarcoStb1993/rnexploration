/*
 * NodeComparator.h
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#ifndef rsb_nbv_exploration_SRC_NODECOMPARATOR_H_
#define rsb_nbv_exploration_SRC_NODECOMPARATOR_H_

#include "ros/ros.h"
#include <rsb_nbv_exploration_msgs/Graph.h>
#include <rsb_nbv_exploration_msgs/Node.h>

namespace rsb_nbv_exploration {

/**
 * @brief Structure to store the node and the particular reward function
 */
struct CompareStruct {
	int node;
	double reward_function;

	/**
	 * @brief Constructor to initialize struct with node index in graph list
	 */
	CompareStruct(int n) {
		node = n;
		reward_function = 0.0;
	}

	/**
	 * @brief Constructor to initialize struct with node index in graph list and reward function
	 */
	CompareStruct(int n, double rf) {
		node = n;
		reward_function = rf;
	}
};

/**
 * @brief Class to maintain the list of nodes ordered by reward function
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
	 * @brief Sort list and recalculate reward functions if necessary
	 * @param Current graph
	 */
	void maintainList(rsb_nbv_exploration_msgs::Graph &rrg);

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
	 * @brief Get the node with the best reward function
	 * @return Index of the node with the reward function in the tree
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
	 * @brief Triggered by a recalculation of all node's path distances and/or heading and a
	 * starts a sorting of the list
	 */
	void robotMoved();

	/**
	 * @brief Triggers sorting of the list
	 */
	void setSortList();

	/**
	 * @brief Retrieve a list of all nodes currently ordered by the reward function
	 * @return List of all nodes managed by this class
	 */
	std::vector<int> getListOfNodes();

private:
	/**
	 * @brief All nodes (their position in the rrg node list) and their respective reward function ordered ascending
	 */
	std::list<CompareStruct> _nodes_ordered_by_reward;
	/**
	 * @brief Should the list be sorted or not
	 */
	bool _sort_list;
	/**
	 * @brief Did the robot move or not? Implies that all reward functions must be recalculated
	 */
	bool _robot_moved;

	/**
	 * @brief Sorts list of nodes with a reward function, the node with the highest reward function comes first
	 */
	void sortByReward();

	/**
	 * @brief Calculates the reward function of each node in the list of nodes
	 * @param Current graph
	 */
	void calculateRewardFunctions(rsb_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Returns the reward function of the given node index
	 * @param Node index of which the reward function should be returned
	 * @return Reward function for given node
	 */
	double getNodeRewardFunction(int node);

	/**
	 * @brief Compares the two given nodes and returns true if the first node's reward function is better
	 * than the second
	 * @param First node
	 * @param Second node
	 * @param Returns if the first node's reward function is better than the second node's reward
	 * function
	 */
	bool compareNodeByReward(const CompareStruct &node_one,
			const CompareStruct &node_two);
};

} /* namespace rsb_nbv_exploration */

#endif /* rsb_nbv_exploration_SRC_NODECOMPARATOR_H_ */
