/*
 * ShortestFrontierConnectionStruct.h
 *
 *  Created on: Apr 2, 2022
 *      Author: marco
 */

#ifndef RRG_NBV_EXPLORATION_INCLUDE_RRG_NBV_EXPLORATION_SHORTESTFRONTIERCONNECTIONSTRUCT_H_
#define RRG_NBV_EXPLORATION_INCLUDE_RRG_NBV_EXPLORATION_SHORTESTFRONTIERCONNECTIONSTRUCT_H_

namespace rrg_nbv_exploration {
/**
 * @brief Structure to store a connection through the local graph between two connecting nodes
 * to frontiers including the nodes in the path and the distance of the path
 */
struct ShortestFrontierConnectionStruct {
	int connecting_node_one;
	int connecting_node_two;
	std::vector<int> path;
	double path_length;

	ShortestFrontierConnectionStruct(int one, int two, std::vector<int> p,
			double l) {
		connecting_node_one = std::max(one, two);
		connecting_node_two = std::min(one, two);
		path = p;
		path_length = l;
	}
};
}

#endif /* RRG_NBV_EXPLORATION_INCLUDE_RRG_NBV_EXPLORATION_SHORTESTFRONTIERCONNECTIONSTRUCT_H_ */
