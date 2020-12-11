/*
 * RneMode.h
 *
 *  Created on: Nov 4, 2020
 *      Author: marco
 */

#ifndef RRT_NBV_EXPLORATION_INCLUDE_RRT_NBV_EXPLORATION_RNEMODE_H_
#define RRT_NBV_EXPLORATION_INCLUDE_RRT_NBV_EXPLORATION_RNEMODE_H_

namespace rrt_nbv_exploration {

/**
 * @brief Operating mode of RNE (classic=0, receding=1, horizon=2, receding_horizon=3)
 */
enum RneMode : int {
	classic = 0, receding = 1, horizon = 2, receding_horizon = 3
};

}

#endif /* RRT_NBV_EXPLORATION_INCLUDE_RRT_NBV_EXPLORATION_RNEMODE_H_ */
