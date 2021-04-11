/*
 * NodeComparator.h
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#ifndef RRT_NBV_EXPLORATION_SRC_NODECOMPARATOR_H_
#define RRT_NBV_EXPLORATION_SRC_NODECOMPARATOR_H_

#include "ros/ros.h"
#include <rrt_nbv_exploration_msgs/Frontiers.h>
#include <rrt_nbv_exploration_msgs/Frontier.h>
#include <stack>

#include <fstream>

namespace rrt_nbv_exploration {

/**
 * @brief Structure to store the frontier and the particular gain-cost-ratio
 */
struct CompareStruct {
	int frontier;
	double gain_cost_ratio;

	/**
	 * @brief Constructor to initialize struct with frontier index in tree list
	 */
	CompareStruct(int n) {
		frontier = n;
		gain_cost_ratio = 0.0;
	}

	/**
	 * @brief Constructor to initialize struct with frontier index in tree list and gain-cost-ratio
	 */
	CompareStruct(int n, double gcr) {
		frontier = n;
		gain_cost_ratio = gcr;
	}
};

/**
 * @brief Class to maintain the list of frontiers ordered by gain-cost-ratio
 */
class FrontierComparator {
public:
	FrontierComparator();
	~FrontierComparator();

	/**
	 * @brief Initialize variables and maintained list
	 */
	void initialization();

	/**
	 * @brief Sort list and recalculate gain-cost-ratios if necessary
	 * @param Current frontiers
	 */
	void maintainList(rrt_nbv_exploration_msgs::Frontiers &frontiers);

	/**
	 * @brief Clear the maintained list of ordered frontiers
	 */
	void clear();

	/**
	 * @brief Add a new frontier to the maintained list of ordered frontiers
	 * @param Index of the frontier in the tree
	 */
	void addFrontier(int frontier);

	/**
	 * @brief Remove the given frontier from the maintained list of ordered frontiers
	 * @param Index of the frontier in the tree
	 */
	void removeFrontier(int frontier);

	/**
	 * @brief Get the frontier with the best cost-gain-ratio
	 * @return Index of the frontier with the best cost-gain-ratio in the tree
	 */
	int getBestFrontier();

	/**
	 * @brief Get the number of frontiers currently in the list
	 * @return The size of the list
	 */
	int getListSize();

	/**
	 * @brief Check if the maintained list of ordered frontiers is empty
	 * @return If the list is empty or not
	 */
	bool isEmpty();

	/**
	 * @brief Triggers a recalculation of all frontier's path distances and a sorting of the list
	 */
	void robotMoved();

	/**
	 * @brief Triggers sorting of the list
	 */
	void setSortList();

private:
	/**
	 * @brief All frontiers (their position in the frontier list) and their respective gain-cost-ratio ordered ascendingly
	 */
	std::list<CompareStruct> _frontiers_ordered_by_gcr;
	/**
	 * @brief Should the list be sorted or not
	 */
	bool _sort_list;
	/**
	 * @brief Did the robot move or not? Implies that all gain-cost-ratios must be recalculated
	 */
	bool _robot_moved;

	/**
	 * @brief Sorts list of frontiers with a gain function which depends on the RNE mode, the frontier with the best
	 * gain-cost-ratio comes first
	 * @param Current tree
	 */
	void sortByGain(rrt_nbv_exploration_msgs::Frontiers &frontiers);

	/**
	 * @brief Calculates the gain-cost-ratio of each frontier in the list of frontiers
	 * @param Current frontiers
	 */
	void calculateGainCostRatios(rrt_nbv_exploration_msgs::Frontiers &frontiers);

	/**
	 * @brief Returns the gain-cost-ratio of the given frontier index
	 * @param Frontier index of which the gain-cost-ratio should be returned
	 * @return Gain-cost-ratio for given frontier
	 */
	double getFrontierGainCostRatio(int frontier);

	/**
	 * @brief Compares the two given frontiers and returns true if the first frontier's gain-cost-ratio is better than the second
	 * @param First frontier
	 * @param Second frontier
	 * @param Returns if the first frontier's gain-cost-ratio is better than the second frontier's gain-cost-ratio
	 */
	bool compareFrontierByRatios(const CompareStruct &frontier_one,
			const CompareStruct &frontier_two);

};

} /* namespace rrt_nbv_exploration */

#endif /* RRT_NBV_EXPLORATION_SRC_NODECOMPARATOR_H_ */
