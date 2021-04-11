/**
Copyright (c) 2016-2018, Paul Bovbel
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the frontier_exploration nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PAUL BOVBEL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include <rrt_nbv_exploration_msgs/Frontier.h>
#include <rrt_nbv_exploration_msgs/Frontiers.h>

namespace rrt_nbv_exploration
{

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch()
  {
  }

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   * @param Minimum frontier size in m
   */
  FrontierSearch(costmap_2d::Costmap2D* costmap, double min_frontier_size);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<rrt_nbv_exploration_msgs::Frontier> searchFrom(geometry_msgs::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  rrt_nbv_exploration_msgs::Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);


private:
  costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double min_frontier_size_;
};
}
#endif
